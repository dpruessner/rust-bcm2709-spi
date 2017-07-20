// rust-bcm-2709-spi
//
// Low-performance SPI interface to BCM2709 hardware peripherals via mmap(...) and direct memory I/O.
//

extern crate libc;
extern crate vcell;


/// A peripheral
#[derive(Debug)]
pub struct Peripheral<T>
where
    T: 'static,
{
    address: *mut T,
}

impl<T> Peripheral<T> {
    /// Creates a new peripheral
    ///
    /// `address` is the base address of the register block
    pub unsafe fn new(address: usize) -> Self {
        Peripheral { address: address as *mut T }
    }

    pub fn borrow<'a>(&self) -> &'a mut T {
        unsafe { &mut *self.get() }
    }

    /// Returns a pointer to the register block
    pub fn get(&self) -> *mut T {
        self.address as *mut T
    }
}

const MAP_BLOCK_SIZE: usize = 4 * 1024;
const SPI_BASE_OFFSET: usize = 0x3f20_4000;
const GPIO_BASE_OFFSET: usize = 0x3f20_0000;


// SPI will be initialized to single-byte operation: each Read/Write to FIFO will dequeue/enqueue
// one byte into the FIFO (LSB).
//
#[doc = "SPI hardware implementation; borrowed from constructs from Japaric's SVD2Rust MCU rust-code generator"]
pub mod spi {
    use vcell::VolatileCell;
    use super::{ libc, SPI_BASE_OFFSET, MAP_BLOCK_SIZE };

    pub struct SPIInterface {
        #[doc = r"bit-1: CPOL, bit-0: CPHA;  SPI mode for the device"]
        pub mode: u8,
        #[doc = r"RaspberryPi has 2 chip select lines for this SPI device. Set this to the device number to drive"]
        pub device: u8,

        address: usize,
    }

    #[repr(C)]
    pub struct SPI {
        cs: CS,
        fifo: FIFO,
        clk: CLK,
    }

    impl SPIInterface {
        pub fn new(fd: i32) -> Self {
            let ptr;
            unsafe {
                ptr = libc::mmap( 0 as *mut libc::c_void, 
                                  MAP_BLOCK_SIZE, 
                                  libc::PROT_READ | libc::PROT_WRITE,
                                  libc::MAP_SHARED,
                                  fd,
                                  SPI_BASE_OFFSET as libc::off_t);
            }
            return SPIInterface { 
                mode: 0,
                device: 0,
                address: ptr as usize, 
            };
        }

        pub fn get<'a>(&'a mut self) -> &'a mut SPI {
            let ptr: *mut SPI = self.address as *mut SPI;
            unsafe{ return &mut *ptr; }
        }

        pub fn set_clock(&mut self, clk: u16) {
            let spi = self.get();
            spi.clk.register.set(clk as u32);
        }


        #[inline(always)]
        pub fn start_transaction(&mut self) {
            let device = self.device;
            let mode = self.mode;

            let spi = self.get();
            let bits = cs::CLEAR_RX_FIFO | cs::CLEAR_TX_FIFO | cs::TA | 
                ((( mode & 0x3) as u32) << 2) | 
                (( device & 0x3) as u32);
            spi.cs.register.set( bits );
        }


        #[inline(always)]
        pub fn stop_transaction(&mut self) {
            let spi = self.get();
            
            // Clear out any pending data
            spi.cs.wait_for_done();        

            let bits = spi.cs.register.get() & !cs::TA;
            spi.cs.register.set(bits);
        }

        #[inline(always)]
        pub fn write_byte(&mut self, byte: u8) -> u8 {
            let spi = self.get();

            // Clear out any pending data
            spi.cs.wait_for_done();        

            spi.fifo.register.set( byte as u32 );

            // Clear out any pending data
            spi.cs.wait_for_done();        

            return (spi.fifo.register.get() & 0xFF) as u8;;
        }
    }
    
    impl Drop for SPIInterface {
        fn drop(&mut self) {
            unsafe {
                libc::munmap( self.address as *mut libc::c_void, MAP_BLOCK_SIZE as libc::size_t );
            }
        }
    }


    pub struct CS {
        register: VolatileCell<u32>,
    }

    pub mod cs {
        pub const RXF: u32 = (1 << 20);
        pub const RXR: u32 = (1 << 19);
        pub const TXD: u32 = (1 << 18);
        pub const RXD: u32 = (1 << 17);
        pub const DONE: u32 = (1 << 16);
        pub const TA: u32  = (1 << 7);
        pub const CLEAR_RX_FIFO: u32 = (1 << 5);
        pub const CLEAR_TX_FIFO: u32 = (1 << 4);
        pub const CPOL: u32 = (1 << 3);
        pub const CPHA: u32 = (1 << 2);

        #[doc = r"Value read from the register"]
        pub struct R {
            bits: u32,
        }
        #[doc = r"Value to write to the register"]
        pub struct W {
            bits: u32,
        }
        impl super::CS {
            #[doc = r"Modifies the content of the CS register"]
            #[inline(always)]
            pub fn modify<F>(&self, f: F)
            where
                for<'w> F: FnOnce(&R, &'w W) -> &'w mut W,
            {
                let bits = self.register.get();
                let r = R { bits: bits };
                let mut w = W { bits: bits };
                f(&r, &mut w);
                self.register.set(w.bits);
            }
            #[doc = r"Reads the contents of the CS register"]
            #[inline(always)]
            pub fn read(&self) -> R {
                R { bits: self.register.get() }
            }
            #[doc = r"Writes to the register"]
            #[inline(always)]
            pub fn write<F>(&self, f: F)
            where
                F: FnOnce(&mut W) -> &mut W,
            {
                let mut w = W::reset_value();
                f(&mut w);
                self.register.set(w.bits);
            }
            #[doc = r"Writes reset value to the register"]
            #[inline(always)]
            pub fn reset(&self) {
                self.write(|w| w)
            }


            #[doc = r"Waits for the DONE bit to be set; will wait indefinitely"]
            #[inline(always)]
            pub fn wait_for_done(&self) {
                while (self.register.get() & DONE) == 0 {
                }
            }

        }
        #[doc = r"value of the field"]
        pub struct CSR {
            bits: u32,
        }
        impl CSR {
            #[doc = r"Value of the field as raw bits"]
            #[inline(always)]
            pub fn bits(&self) -> u32 {
                self.bits
            }
        }
        #[doc = r"Proxy"]
        pub struct _CSW<'a> {
            w: &'a mut W,
        }
        impl<'a> _CSW<'a> {
            #[doc = r"Writes raw bits to the field"]
            #[inline(always)]
            pub unsafe fn bits(self, value:u32) -> &'a mut W {
                const MASK: u32 = 0xFFFF_FFFF;
                const OFFSET: u8 = 0;
                self.w.bits &= !((MASK as u32) << OFFSET);
                self.w.bits |= ((value & MASK) as u32) << OFFSET;
                self.w
            }
        }
        impl R {
            #[doc = r"Value of the register as raw bits"]
            #[inline(always)]
            pub fn bits(&self) -> u32 {
                self.bits
            }
            #[doc = "Bits 0:31 - Control status register bits"]
            #[inline(always)]
            pub fn csr(&self) -> CSR {
                let bits = {
                    const MASK: u32 = 0xFFFF_FFFF;
                    const OFFSET: u8 = 0;
                    ((self.bits >> OFFSET) & MASK as u32) as u32
                };
                CSR { bits }
            }
        }

        impl W {
            #[doc = r"Reset value of the register"]
            #[inline(always)]
            pub fn reset_value() -> W {
                W { bits: (1 << 12) as u32 }
            }
            #[doc = r"Write raw bits to the register"]
            #[inline(always)]
            pub fn bits(&mut self, bits: u32) -> &mut Self {
                self.bits = bits;
                self
            }
            #[doc = r"Bits 0:31 - Control status register bits"]
            #[inline(always)]
            pub fn cs(&mut self) -> _CSW {
                _CSW { w: self }
            }
        }
    }

    pub struct FIFO {
        register: VolatileCell<u32>,
    }
    pub mod fifo {
        #[doc = r"Value read from the register"]
        pub struct R {
            bits: u32,
        }
        #[doc = r"Value to write to the register"]
        pub struct W {
            bits: u32,
        }

        impl super::FIFO {
            #[doc = r"Modifies the content of the FIFO register"]
            #[inline(always)]
            pub fn modify<F>(&self, f: F)
            where
                for<'w> F: FnOnce(&R, &'w W) -> &'w mut W,
            {
                let bits = self.register.get();
                let r = R { bits: bits };
                let mut w = W { bits: bits };
                f(&r, &mut w);
                self.register.set(w.bits);
            }
            #[doc = r"Reads the contents of the FIFO register"]
            #[inline(always)]
            pub fn read(&self) -> R {
                R { bits: self.register.get() }
            }
            #[doc = r"Writes to the register"]
            #[inline(always)]
            pub fn write<F>(&self, f: F)
            where
                F: FnOnce(&mut W) -> &mut W,
            {
                let mut w = W::reset_value();
                f(&mut w);
                self.register.set(w.bits);
            }
            #[doc = r"Writes reset value to the register"]
            #[inline(always)]
            pub fn reset(&self) {
                self.write(|w| w)
            }

        }
        #[doc = r"value of the field"]
        pub struct FIFOR {
            bits: u32,
        }
        impl FIFOR {
            #[doc = r"Value of the field as raw bits"]
            #[inline(always)]
            pub fn bits(&self) -> u32 {
                self.bits
            }
        }
        #[doc = r"Proxy"]
        pub struct _FIFOW<'a> {
            w: &'a mut W,
        }
        impl<'a> _FIFOW<'a> {
            #[doc = r"Writes raw bits to the field"]
            #[inline(always)]
            pub unsafe fn bits(self, value:u32) -> &'a mut W {
                const MASK: u32 = 0xFFFF_FFFF;
                const OFFSET: u8 = 0;
                self.w.bits &= !((MASK as u32) << OFFSET);
                self.w.bits |= ((value & MASK) as u32) << OFFSET;
                self.w
            }
        }
        impl R {
            #[doc = r"Value of the register as raw bits"]
            #[inline(always)]
            pub fn bits(&self) -> u32 {
                self.bits
            }
            #[doc = "Bits 0:31 - Control status register bits"]
            #[inline(always)]
            pub fn fifor(&self) -> FIFOR  {
                let bits = {
                    const MASK: u32 = 0xFFFF_FFFF;
                    const OFFSET: u8 = 0;
                    ((self.bits >> OFFSET) & MASK as u32) as u32
                };
                FIFOR { bits }
            }
        }
        impl W {
            #[doc = r"Reset value of the register"]
            #[inline(always)]
            pub fn reset_value() -> W {
                W { bits: 0 as u32 }
            }
            #[doc = r"Write raw bits to the register"]
            #[inline(always)]
            pub unsafe fn bits(&mut self, bits: u32) -> &mut Self {
                self.bits = bits;
                self
            }
            #[doc = r"Bits 0:31 - Control status register bits"]
            #[inline(always)]
            pub fn fifo(&mut self) -> _FIFOW {
                _FIFOW { w: self }
            }
        }
    }

    pub struct CLK {
        register: VolatileCell<u32>,
    }
    pub mod clk {
        #[doc = r"Value read from the register"]
        pub struct R {
            bits: u32,
        }
        #[doc = r"Value to write to the register"]
        pub struct W {
            bits: u32,
        }

        impl super::CLK {
            #[doc = r"Modifies the content of the FIFO register"]
            #[inline(always)]
            pub fn modify<F>(&self, f: F)
            where
                for<'w> F: FnOnce(&R, &'w W) -> &'w mut W,
            {
                let bits = self.register.get();
                let r = R { bits: bits };
                let mut w = W { bits: bits };
                f(&r, &mut w);
                self.register.set(w.bits);
            }
            #[doc = r"Reads the contents of the FIFO register"]
            #[inline(always)]
            pub fn read(&self) -> R {
                R { bits: self.register.get() }
            }
            #[doc = r"Writes to the register"]
            #[inline(always)]
            pub fn write<F>(&self, f: F)
            where
                F: FnOnce(&mut W) -> &mut W,
            {
                let mut w = W::reset_value();
                f(&mut w);
                self.register.set(w.bits);
            }
            #[doc = r"Writes reset value to the register"]
            #[inline(always)]
            pub fn reset(&self) {
                self.write(|w| w)
            }

        }

        #[doc = r"value of the field"]
        pub struct CLKR {
            bits: u32,
        }
        impl CLKR {
            #[doc = r"Value of the field as raw bits"]
            #[inline(always)]
            pub fn bits(&self) -> u32 {
                self.bits
            }
        }
        #[doc = r"Proxy"]
        pub struct _CLKW<'a> {
            w: &'a mut W,
        }
        impl<'a> _CLKW<'a> {
            #[doc = r"Writes raw bits to the field"]
            #[inline(always)]
            pub unsafe fn bits(self, value:u32) -> &'a mut W {
                const MASK: u32 = 0x0000_FFFF;
                const OFFSET: u8 = 0;
                self.w.bits &= !((MASK as u32) << OFFSET);
                self.w.bits |= ((value & MASK) as u32) << OFFSET;
                self.w
            }
        }
        impl R {
            #[doc = r"Value of the register as raw bits"]
            #[inline(always)]
            pub fn bits(&self) -> u32 {
                self.bits
            }
            #[doc = "Bits 0:31 - Control status register bits"]
            #[inline(always)]
            pub fn clkr(&self) -> CLKR  {
                let bits = {
                    const MASK: u32 = 0x0000_FFFF;
                    const OFFSET: u8 = 0;
                    ((self.bits >> OFFSET) & MASK as u32) as u32
                };
                CLKR { bits }
            }
        }
        impl W {
            #[doc = r"Reset value of the register"]
            #[inline(always)]
            pub fn reset_value() -> W {
                W { bits: 0 as u32 }
            }
            #[doc = r"Write raw bits to the register"]
            #[inline(always)]
            pub unsafe fn bits(&mut self, bits: u32) -> &mut Self {
                self.bits = bits;
                self
            }
            #[doc = r"Bits 0:31 - Control status register bits"]
            #[inline(always)]
            pub fn clk(&mut self) -> _CLKW {
                _CLKW { w: self }
            }
        }
    }
}


pub struct DirectMemory {
    fd: i32,
}

// 
// Lifetimes and mmap(...) access:
// -------------------------------
//
// On file descriptor close, memory is not unmapped.  From POSIX, the memory mapped from a file
// can continue to be used until it is unmapped, even if the file is closed before the unmap.
//
// This means that the GPIO and SPI (mmapped access) to _/dev/mem_ can remain active and call
// **munmap(...)** when Dropped, which will fully close out the resource.
//
//
impl DirectMemory {
    pub fn get() -> Result<Self, &'static str> {
        let fd;
        unsafe {
            fd = libc::open( "/dev/mem".as_ptr() as *const u8, libc::O_RDWR | libc::O_SYNC | libc::O_CLOEXEC );
        }
        if fd < 0 {
            return Err("Could not open /dev/mem");
        }

        Ok( DirectMemory { 
            fd: fd, 
            } )
    }

    pub fn gpio(&mut self) -> Result<gpio::GPIOInterface, &'static str> {
        return Ok(gpio::GPIOInterface::new( self.fd ) );
    }

    #[doc = r"Creates the SPI interface; takes ownership via GPIO of all SPI pins: SPI_CE1, SPI_CE0, SPI_MISO, SPI_MOSI, SPI_CLK"]
    pub fn spi(&mut self) -> Result<spi::SPIInterface, &'static str> {
        let mut gpio_per = self.gpio()?;
        let gpio_if = gpio_per.get();
        gpio_if.fsel( 7, gpio::FSEL_0 );  // SPI_CE1
        gpio_if.fsel( 8, gpio::FSEL_0 );  // SPI_CE0
        gpio_if.fsel( 9, gpio::FSEL_0 );  // SPI_MISO
        gpio_if.fsel(10, gpio::FSEL_0 );  // SPI_MOSI
        gpio_if.fsel(11, gpio::FSEL_0 );  // SPI_CLK

        return Ok(spi::SPIInterface::new(self.fd));
    }
}

impl Drop for DirectMemory {
    fn drop(&mut self) {
        if self.fd == 0 {
            return;
        }
        unsafe {
            libc::close( self.fd );
        }
    }
}



pub mod gpio {
    use vcell::VolatileCell;
    use super::{libc, MAP_BLOCK_SIZE, GPIO_BASE_OFFSET};

    pub const FSEL_INPUT: u8 = 0;
    pub const FSEL_OUTPUT: u8 = 1;
    pub const FSEL_0: u8 = 0b100;
    pub const FSEL_1: u8 = 0b101;
    pub const FSEL_2: u8 = 0b110;
    pub const FSEL_3: u8 = 0b111;
    pub const FSEL_4: u8 = 0b011;
    pub const FSEL_5: u8 = 0b010;

    pub struct GPIOInterface {
        address: usize,
    }

    impl GPIOInterface {

        pub fn get<'b>(&'b mut self) -> &'b mut GPIO {
            let ptr: *mut GPIO = self.address as *mut GPIO;
            unsafe { return &mut *ptr; }
        }

        pub fn new(fd: i32) -> Self {
            unsafe {
                let ptr = libc::mmap( 0 as *mut libc::c_void, 
                                      MAP_BLOCK_SIZE, 
                                      libc::PROT_READ | libc::PROT_WRITE, 
                                      libc::MAP_SHARED, 
                                      fd, 
                                      GPIO_BASE_OFFSET as libc::off_t);
                return GPIOInterface { 
                    address: ptr as usize, 
                };
            }
        }
    }
    impl Drop for GPIOInterface {
        fn drop(&mut self) {
            unsafe {
                libc::munmap( self.address as *mut libc::c_void, MAP_BLOCK_SIZE as libc::size_t );
            }
        }
    }


    #[repr(C)]
    pub struct GPIO {
        gpfsel: [VolatileCell<u32>; 6],
        res0: u32,
        gpset: [VolatileCell<u32>; 2],
        res1: u32,
        gpclr: [VolatileCell<u32>; 2],

    }

    impl GPIO {
        #[inline(always)]
        pub fn fsel(&mut self, pin: usize, mode: u8 ) {
            let bank = pin / 10 as usize;
            let mut val =  self.gpfsel[bank].get() ;
            let shift = (pin - (10 * bank) ) * 3;
            let mask = !((0x07 << shift) as u32);
            val = (val & mask) | ((mode as u32) << shift);
            self.gpfsel[bank].set(val);
        }
        #[inline(always)]
        pub fn pin_high( &mut self, pin: usize ) {
            let bank = pin / 32 as usize;
            let shift = pin - (32 * bank) ;
            let val = 1 << shift;
            self.gpset[bank].set(val);
        }
        #[inline(always)]
        pub fn pin_low(&mut self, pin: usize) {
            let bank = pin / 32 as usize;
            let shift = pin - (32 * bank) ;
            let val = 1 << shift;
            self.gpclr[bank].set(val);
        }

    }


}



//pub struct SPI {
//    fd: i32,
//    gpio_mem: *mut u8,
//    spi_mem: *mut u8,
//    spi: Peripheral<spi::SPIPeripheral>,
//}
//
//impl Drop for SPI {
//    fn drop(&mut self) {
//    }
//}
//
//impl SPI {
//    pub fn get() -> Option<Self> {
//        None
//    }
//}
//


    

#[cfg(test)]
mod tests {

    use super::{DirectMemory, gpio };

    #[test]
    #[allow(dead_code)]
    #[allow(unused_variables)]
    fn smoke_direct_memory() {
        let mem = match DirectMemory::get() {
            Ok(mem) => mem,
            Err(reason) => {
                println!("somke_direct_memory(): error: {}", reason);
                return;
            }
        };
        println!("smoke_direct_memory: success.");
    }

    #[test]
    #[allow(dead_code)]
    #[allow(unused_variables)]
    fn test_gpio_create() {
        let mut mem = DirectMemory::get().unwrap();
        let gpio = mem.gpio();
    }

    #[test]
    #[cfg(feature = "disabled")]    // Uncomment to verify borrow rules for mmap(...) memory reference of GPIO structure
    fn test_gpio_shouldnt_compile() {
        let mut the_gpio: &mut gpio::GPIO;
        {
            let mut mem = DirectMemory::get().unwrap();
            let mut gpio_iface = mem.gpio().unwrap();
            the_gpio = gpio_iface.get();
        }
        the_gpio.fsel( 26, gpio::FSEL_OUTPUT );
    }

    #[test]
    fn test_gpio_fsel_out() {
        let mut gpio_iface;
        {
            let mut mem = DirectMemory::get().unwrap();
            gpio_iface = mem.gpio().unwrap();
        }
        {
            let mut the_gpio: &mut gpio::GPIO;
            the_gpio = gpio_iface.get();
            the_gpio.fsel( 26, gpio::FSEL_OUTPUT );
        }
    }
    #[test]
    fn test_gpio_fsel_in() {
        let mut mem = DirectMemory::get().unwrap();
        let mut gpio_iface = mem.gpio().unwrap();
        let mut the_gpio = gpio_iface.get();
        the_gpio.fsel( 26, gpio::FSEL_INPUT );
    }
    #[test]
    fn test_gpio_set_low() {
        let mut mem = DirectMemory::get().unwrap();
        let mut gpio_iface = mem.gpio().unwrap();
        let mut the_gpio = gpio_iface.get();
        the_gpio.pin_low( 26 );
    }
    #[test]
    fn test_gpio_set_high() {
        let mut mem = DirectMemory::get().unwrap();
        let mut gpio_iface = mem.gpio().unwrap();
        let mut the_gpio = gpio_iface.get();
        the_gpio.pin_high( 26 );
    }

    #[test]
    #[allow(dead_code)]
    fn test_spi_init() {
        let mut mem = DirectMemory::get().unwrap();
        #[allow(unused_variables)]
        let spi_iface = mem.spi();

    }

    #[test]
    fn test_spi_write() {
        println!("test_spi_write()");
        let mut mem = DirectMemory::get().unwrap();
        let mut spi_iface = mem.spi().unwrap();
        spi_iface.set_clock(32);
        spi_iface.start_transaction();
        spi_iface.write_byte( b'A' );
        spi_iface.stop_transaction();
        println!("done");
    }
}
