//go:build esp32

package machine

// Parallel camera input (DVP) with I2S0 in camera mode and DMA. It reads JPEG pictures.
// See ESP32 TRM v5.8 section 22.5.2 and esp32-camera target/esp32/ll_cam.c.

import (
	"device/esp"
	"errors"
	"runtime/interrupt"
	"runtime/volatile"
	"unsafe"
)

// GPIO matrix input signals of I2S0. See ESP-IDF soc/esp32/include/soc/gpio_sig_map.h.
const (
	sigI2S0WSIn     = 28
	sigI2S0Data0In  = 140
	sigI2S0HSyncIn  = 190
	sigI2S0VSyncIn  = 191
	sigI2S0HEnable  = 192
	gpioConstantOne = 0x38
)

// cpuInterruptFromI2S0 is a free level triggered CPU interrupt.
// See ESP32 TRM v5.8 table 8.3-2.
const cpuInterruptFromI2S0 = 18

// dmaDesc is a DMA descriptor. It must be in internal RAM.
// See ESP32 TRM v5.8 section 2.3.2.
type dmaDesc struct {
	ctrl uint32
	buf  uintptr
	next uintptr
}

const dmaOwnerDMA = 1 << 31

// The DMA ring has 4 chunks of 512 words. In JPEG mode each word holds one byte.
const (
	dvpChunks     = 4
	dvpChunkWords = 512
)

var dvpRing [dvpChunks][dvpChunkWords]uint32

// When the buffer is in PSRAM, the interrupt puts chunks in this FIFO in internal RAM
// and Capture moves them on. Writing PSRAM in the interrupt is too slow.
const dvpFIFOChunks = 8

var dvpFIFO [dvpFIFOChunks][dvpChunkWords / 4]uint32

// The PSRAM window of the data bus. See ESP32 TRM v5.8 section 4.3.2.2.
const (
	dvpPSRAMStart = 0x3F800000
	dvpPSRAMEnd   = 0x3FC00000
)

const (
	dvpIdle = iota
	dvpArmed
	dvpRunning
	dvpDone
)

// DVPConfig holds the pins of a parallel camera.
type DVPConfig struct {
	Data  [8]Pin // D0 to D7
	PCLK  Pin    // pixel clock
	VSYNC Pin    // low at the start of each picture
	HREF  Pin    // high while a line of data comes
}

// DVP is a parallel camera input.
type DVP struct {
	cfg   DVPConfig
	ring  [dvpChunks]dmaDesc
	state uint32
	dst   []byte
	fifo  bool
	head  uint32 // chunks taken from DMA
	tail  uint32 // chunks moved from the FIFO to dst
	err   error
}

// DVP0 is the camera input on I2S0.
var DVP0 = &DVP{}

var (
	errDVPPin      = errors.New("dvp: pin missing or used twice")
	errDVPTimeout  = errors.New("dvp: timeout")
	errDVPTooSmall = errors.New("dvp: picture does not fit in the buffer")
	errDVPNoStart  = errors.New("dvp: no JPEG start marker")
	errDVPNoEnd    = errors.New("dvp: no JPEG end marker")
	errDVPOverrun  = errors.New("dvp: data lost")
)

// Configure sets up the pins and I2S0 in camera mode.
func (d *DVP) Configure(cfg DVPConfig) error {
	var pins [11]Pin
	copy(pins[:], cfg.Data[:])
	pins[8], pins[9], pins[10] = cfg.PCLK, cfg.VSYNC, cfg.HREF
	var used uint64
	for _, p := range pins {
		if p >= maxPin || used&(1<<p) != 0 {
			return errDVPPin
		}
		used |= 1 << p
	}
	d.cfg = cfg

	for i, p := range cfg.Data {
		p.Configure(PinConfig{Mode: PinInput})
		inFunc(sigI2S0Data0In + uint32(i)).Set(esp.GPIO_FUNC_IN_SEL_CFG_SEL | uint32(p))
	}
	for _, s := range [...]struct {
		pin Pin
		sig uint32
	}{{cfg.PCLK, sigI2S0WSIn}, {cfg.VSYNC, sigI2S0VSyncIn}, {cfg.HREF, sigI2S0HSyncIn}} {
		s.pin.Configure(PinConfig{Mode: PinInput})
		inFunc(s.sig).Set(esp.GPIO_FUNC_IN_SEL_CFG_SEL | uint32(s.pin))
	}
	inFunc(sigI2S0HEnable).Set(esp.GPIO_FUNC_IN_SEL_CFG_SEL | gpioConstantOne)

	esp.DPORT.SetPERIP_CLK_EN_I2S0_CLK_EN(1)
	esp.DPORT.SetPERIP_RST_EN_I2S0_RST(0)

	i2s := esp.I2S0
	d.reset()
	i2s.SetCONF_RX_SLAVE_MOD(1)
	i2s.SetCONF_RX_RIGHT_FIRST(0)
	i2s.SetCONF_RX_MSB_RIGHT(0)
	i2s.SetCONF_RX_MSB_SHIFT(0)
	i2s.SetCONF_RX_MONO(0)
	i2s.SetCONF_RX_SHORT_SYNC(0)
	i2s.SetCONF2_LCD_EN(1)
	i2s.SetCONF2_CAMERA_EN(1)
	i2s.SetCLKM_CONF_CLKM_DIV_A(0)
	i2s.SetCLKM_CONF_CLKM_DIV_B(0)
	i2s.SetCLKM_CONF_CLKM_DIV_NUM(2)
	i2s.SetFIFO_CONF_DSCR_EN(1)
	i2s.SetFIFO_CONF_RX_FIFO_MOD(3) // one byte in each word, see esp32-camera ll_cam.c
	i2s.SetFIFO_CONF_RX_FIFO_MOD_FORCE_EN(1)
	i2s.SetCONF_CHAN_RX_CHAN_MOD(1)
	i2s.SetSAMPLE_RATE_CONF_RX_BITS_MOD(0)
	i2s.TIMING.Set(0)
	i2s.SetTIMING_RX_DSYNC_SW(1)

	i2s.INT_ENA.Set(esp.I2S_INT_ENA_IN_SUC_EOF_INT_ENA | esp.I2S_INT_ENA_IN_DSCR_ERR_INT_ENA |
		esp.I2S_INT_ENA_IN_DSCR_EMPTY_INT_ENA)
	esp.DPORT.SetPRO_I2S0_INT_MAP(cpuInterruptFromI2S0)
	if err := interrupt.New(cpuInterruptFromI2S0, handleDVPInterrupt).Enable(); err != nil {
		return err
	}
	cfg.VSYNC.SetInterrupt(0, nil)
	return cfg.VSYNC.SetInterrupt(PinFalling, handleDVPVSYNC)
}

func (d *DVP) reset() {
	i2s := esp.I2S0
	i2s.SetCONF_RX_RESET(1)
	i2s.SetCONF_RX_RESET(0)
	i2s.SetCONF_RX_FIFO_RESET(1)
	i2s.SetCONF_RX_FIFO_RESET(0)
	i2s.SetLC_CONF_IN_RST(1)
	i2s.SetLC_CONF_IN_RST(0)
	i2s.SetLC_CONF_AHBM_FIFO_RST(1)
	i2s.SetLC_CONF_AHBM_FIFO_RST(0)
	i2s.SetLC_CONF_AHBM_RST(1)
	i2s.SetLC_CONF_AHBM_RST(0)
}

func (d *DVP) stop() {
	i2s := esp.I2S0
	i2s.SetCONF_RX_START(0)
	i2s.SetIN_LINK_INLINK_STOP(1)
}

// Capture reads the next JPEG picture into buf and returns its length.
// Other goroutines run while it waits.
func (d *DVP) Capture(buf []byte) (int, error) {
	for i := range d.ring {
		d.ring[i] = dmaDesc{
			ctrl: dvpChunkWords*4 | dmaOwnerDMA,
			buf:  uintptr(unsafe.Pointer(&dvpRing[i][0])),
			next: uintptr(unsafe.Pointer(&d.ring[(i+1)%dvpChunks])),
		}
	}
	d.dst, d.head, d.tail, d.err = buf, 0, 0, nil
	addr := uintptr(unsafe.Pointer(unsafe.SliceData(buf)))
	d.fifo = addr >= dvpPSRAMStart && addr < dvpPSRAMEnd
	volatile.StoreUint32(&d.state, dvpArmed)

	end := nanotime() + 1e9
	for volatile.LoadUint32(&d.state) != dvpDone {
		if nanotime() > end {
			mask := interrupt.Disable()
			d.stop()
			volatile.StoreUint32(&d.state, dvpIdle)
			interrupt.Restore(mask)
			d.dst = nil
			return 0, errDVPTimeout
		}
		d.moveFIFO()
		gosched()
	}
	d.moveFIFO()
	volatile.StoreUint32(&d.state, dvpIdle)
	d.dst = nil
	if d.err != nil {
		return 0, d.err
	}
	n := int(d.head) * dvpChunkWords
	if n < 3 || buf[0] != 0xFF || buf[1] != 0xD8 || buf[2] != 0xFF {
		return 0, errDVPNoStart
	}
	// The camera can send padding after the end marker, so search from the start.
	for i := 3; i+1 < n; i++ {
		if i%1024 == 0 {
			gosched()
		}
		if buf[i] == 0xFF && buf[i+1] == 0xD9 {
			return i + 2, nil
		}
	}
	return 0, errDVPNoEnd
}

// handleDVPVSYNC starts DMA at the first picture start and ends the capture at the next.
func handleDVPVSYNC(Pin) {
	d := DVP0
	switch volatile.LoadUint32(&d.state) {
	case dvpArmed:
		i2s := esp.I2S0
		i2s.SetCONF_RX_START(0)
		d.reset()
		i2s.INT_CLR.Set(0xFFFFFFFF)
		// DMA stops at a chunk that was not copied yet. See ESP32 TRM v5.8 register 22.24.
		i2s.SetLC_CONF_CHECK_OWNER(1)
		i2s.SetRXEOF_NUM(dvpChunkWords)
		i2s.SetIN_LINK_INLINK_ADDR(uint32(uintptr(unsafe.Pointer(&d.ring[0]))) & 0xFFFFF)
		i2s.SetIN_LINK_INLINK_START(1)
		i2s.SetCONF_RX_START(1)
		volatile.StoreUint32(&d.state, dvpRunning)
	case dvpRunning:
		d.stop()
		d.drain()
		if volatile.LoadUint32(&d.state) != dvpRunning {
			return
		}
		// Take the last chunk whole. Capture cuts the picture at its end marker.
		if int(d.head+1)*dvpChunkWords <= len(d.dst) && !d.take(&dvpRing[d.head%dvpChunks]) {
			return
		}
		volatile.StoreUint32(&d.state, dvpDone)
	}
}

func handleDVPInterrupt(interrupt.Interrupt) {
	d := DVP0
	if volatile.LoadUint32(&d.state) != dvpRunning {
		esp.I2S0.INT_CLR.Set(0xFFFFFFFF)
		return
	}
	d.drain()
}

// drain takes the chunks that DMA has finished and gives them back to DMA.
func (d *DVP) drain() {
	i2s := esp.I2S0
	raw := i2s.INT_RAW.Get()
	if raw&(esp.I2S_INT_RAW_IN_DSCR_ERR_INT_RAW|esp.I2S_INT_RAW_IN_DSCR_EMPTY_INT_RAW) != 0 {
		d.finish(errDVPOverrun)
		return
	}
	if raw&esp.I2S_INT_RAW_IN_SUC_EOF_INT_RAW == 0 {
		return
	}
	i2s.INT_CLR.Set(esp.I2S_INT_CLR_IN_SUC_EOF_INT_CLR)
	done := int(uintptr(i2s.IN_EOF_DES_ADDR.Get())-uintptr(unsafe.Pointer(&d.ring[0]))) / 12
	for {
		i := d.head % dvpChunks
		if !d.take(&dvpRing[i]) {
			return
		}
		d.ring[i].ctrl = dvpChunkWords*4 | dmaOwnerDMA
		if int(i) == done {
			return
		}
	}
}

// take copies a chunk to dst, or to the FIFO when dst is in PSRAM.
// It runs in the interrupt.
func (d *DVP) take(chunk *[dvpChunkWords]uint32) bool {
	head := d.head
	if int(head+1)*dvpChunkWords > len(d.dst) {
		d.finish(errDVPTooSmall)
		return false
	}
	if !d.fifo {
		dvpCopyChunk(d.dst[head*dvpChunkWords:], chunk)
	} else if head-volatile.LoadUint32(&d.tail) < dvpFIFOChunks {
		dvpCopyChunk(dvpFIFOSlot(head), chunk)
	} else {
		d.finish(errDVPOverrun)
		return false
	}
	volatile.StoreUint32(&d.head, head+1)
	return true
}

// moveFIFO moves the chunks in the FIFO to dst in PSRAM.
// Other goroutines run after each chunk.
func (d *DVP) moveFIFO() {
	if !d.fifo {
		return
	}
	for tail := d.tail; tail != volatile.LoadUint32(&d.head); tail++ {
		copy(d.dst[tail*dvpChunkWords:], dvpFIFOSlot(tail))
		volatile.StoreUint32(&d.tail, tail+1)
		gosched()
	}
}

func dvpFIFOSlot(n uint32) []byte {
	return unsafe.Slice((*byte)(unsafe.Pointer(&dvpFIFO[n%dvpFIFOChunks])), dvpChunkWords)
}

func (d *DVP) finish(err error) {
	d.stop()
	esp.I2S0.INT_CLR.Set(0xFFFFFFFF)
	d.err = err
	volatile.StoreUint32(&d.state, dvpDone)
}

// dvpCopyChunk copies byte 2 of each DMA word, where the camera byte is. It writes
// 4 bytes at a time when it can, which is much faster when dst is in PSRAM.
//
//go:section .iram.dvp
func dvpCopyChunk(dst []byte, chunk *[dvpChunkWords]uint32) int {
	if uintptr(unsafe.Pointer(&dst[0]))%4 == 0 && len(dst) >= dvpChunkWords {
		out := (*[dvpChunkWords / 4]uint32)(unsafe.Pointer(&dst[0]))
		for i := range out {
			w := chunk[4*i : 4*i+4 : 4*i+4]
			out[i] = w[0]>>16&0xFF | w[1]>>8&0xFF00 | w[2]&0xFF0000 | w[3]<<8&0xFF000000
		}
		return dvpChunkWords
	}
	for i, w := range chunk {
		dst[i] = byte(w >> 16)
	}
	return dvpChunkWords
}
