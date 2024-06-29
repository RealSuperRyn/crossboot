#![no_std]
#![no_main]
extern crate alloc;
use alloc::boxed::Box;
use alloc::vec;
use alloc::vec::Vec;
//Big thanks to the creators of the "bootloader" crates, as I needed to reference their UEFI code to know what
//to do here
use core::ptr::{addr_of, addr_of_mut, read};
use crosshw::{elf::structure::ELFInfo, memory::paging::*};
use uefi::{
    allocator::exit_boot_services,
    allocator::Allocator,
    fs::{FileSystem, Path, PathBuf},
    helpers::system_table,
    prelude::*,
    println,
    proto::{
        console::gop::{GraphicsOutput, PixelFormat},
        media::fs::SimpleFileSystem,
    },
    table::{
        boot::{
            MemoryDescriptor, MemoryMap, MemoryType, OpenProtocolAttributes, OpenProtocolParams,
            ScopedProtocol,
        },
        cfg::{ConfigTableEntry, ACPI2_GUID},
    },
    CStr16, Char16,
};
#[cfg(target_arch = "x86_64")]
use x86_64::{
    structures::paging::{frame::PhysFrame, page::AddressNotAligned, FrameAllocator, Size4KiB},
    PhysAddr,
};

#[global_allocator]
static ALLOCATOR: uefi::allocator::Allocator = uefi::allocator::Allocator;

pub struct bootresources<'a> {
    acpi: acpiinfo,
    fb: framebufinfo<'a>,
}

#[derive(Debug, Clone, Copy)]
#[repr(C, packed)]
pub struct rsdp {
    signature: [u8; 8],
    checksum: u8,
    oemid: [u8; 6],
    revision: u8,
    rsdtaddress: u32, //deprecated, but should probably be present
}
#[derive(Debug, Clone, Copy)]
#[repr(C, packed)]
pub struct xsdp {
    signature: [u8; 8],
    checksum: u8,
    oemid: [u8; 6],
    revision: u8,
    rsdtaddress: u32, //redundant here
    length: u32,      //Start of XSDT specific values
    xsdtaddress: u32, //Probably the replacement for rsdtaddress
    extendedchecksum: u8,
    uselesspadding: [u8; 3],
}
#[derive(Debug)]
enum acpiinfo {
    RSDP(rsdp),
    XSDP(xsdp),
}
#[repr(C)]
struct framebufinfo<'a> {
    //"why 8-char names" because i thought it'd be cool B)
    fblength: usize,
    pxformat: &'a [u8; 4],
    fbrwidth: usize,
    fbheight: usize,
    bitsppix: usize,
    fbstride: usize,
}

#[entry]
fn main(_image_handle: Handle, mut system_table: SystemTable<Boot>) -> Status {
    uefi::helpers::init(&mut system_table).unwrap();
    unsafe {
        uefi::allocator::init(&mut system_table);
    } //remember to call uefi::allocator::exit_boot_services() later
    println!("CrossBoot {} blah", env!("CARGO_PKG_VERSION"));
    println!("FIXME: Panics cause an X64 illegal instruction fault. If you see one, it was probably a panic.");
    //First, we obtain the ACPI information.

    let rsdp_ptr = system_table
        .config_table()
        .into_iter()
        .find(|i| i.guid == ACPI2_GUID)
        .unwrap()
        .address;
    let rsdp_table: rsdp = unsafe { *(rsdp_ptr as *mut rsdp) };
    assert!(&rsdp_table.signature == b"RSD PTR ");
    let acpi = if rsdp_table.revision == 2 {
        acpiinfo::XSDP(unsafe { *(rsdp_ptr as *mut xsdp) })
    } else {
        acpiinfo::RSDP(rsdp_table)
    };

    //Second, we get a framebuffer.
    /*
    let fbhandle = system_table
        .boot_services()
        .get_handle_for_protocol::<GraphicsOutput>()
        .ok()
        .unwrap();
    let mut fbprotocol = system_table
        .boot_services()
        .open_protocol_exclusive::<GraphicsOutput>(fbhandle)
        .unwrap();
    let fbmode = fbprotocol
        .modes(&system_table.boot_services())
        .filter(|m| {
            let res = m.info().resolution();
            res.1 >= 1280 && res.0 >= 720 //hardcoded thing to be removed later
        })
        .last()
        .unwrap();
    fbprotocol.set_mode(&fbmode);
    let modeinfo = fbprotocol.current_mode_info();
    let mut fb = fbprotocol.frame_buffer();
    let fbinfo = framebufinfo {
        fblength: fb.size(),
        pxformat: match modeinfo.pixel_format() {
            PixelFormat::Rgb => b"rgb0",
            PixelFormat::Bgr => b"bgr0",
            PixelFormat::Bitmask | PixelFormat::BltOnly => {
                panic!("Bitmask & BltOnly framebuffers unsupported")
            }
        },
        fbrwidth: modeinfo.resolution().0,
        fbheight: modeinfo.resolution().1,
        bitsppix: 4, //We only plan on supporting
        fbstride: modeinfo.stride(),
    };
    */
    let kernelpath = cstr16!("unbusyloop");
    let mut sfs: ScopedProtocol<SimpleFileSystem> = system_table
        .boot_services()
        .get_image_file_system(system_table.boot_services().image_handle())
        .unwrap();
    let mut filesys = FileSystem::new(sfs);
    let mut kernel_file: Box<[u8]> = {
        if !filesys.try_exists(Path::new(kernelpath)).unwrap() {
            panic!("Kernel not in esp root!");
        }
        filesys
            .read(Path::new(kernelpath))
            .unwrap()
            .into_boxed_slice()
    };

    let mut kernel_info: ELFInfo =
        unsafe { ELFInfo::from_ptr(addr_of_mut!((*kernel_file)[0])).unwrap() };
    let kpagecount = (((kernel_file.len() as f64) / 4096.0).ceil()) as usize;
    let kfinalpageend = (kernel_file.len() - ((kpagecount - 1) * 4096)) - 1;
    println!(
        "Kernel pages: {}\nKernel final page end index: {}",
        kpagecount, kfinalpageend
    );
    let mut fralloc = UEFIFrameAlloc {
        pages: FreeList::new(),
        physpagecount: 0,
    };

    //NO ALLOC FEATURES PAST THIS POINT!!!!

    fralloc.init();
    exit_boot_services();
    let uefipayload = unsafe {
        system_table
            .unsafe_clone()
            .exit_boot_services(MemoryType::BOOT_SERVICES_DATA);
    };
    let pcount = fralloc.physpagecount;

    let mut page_manager: PageHierarchy = unsafe { PageHierarchy::new(&mut fralloc, pcount) };
    loop {}
    let kernel_offset = 0xC0000000;

    for i in 0..kpagecount - 1 {
        let ptr =
            page_manager.get_table_at_vaddr(&mut fralloc, kernel_offset + (0x1000 * i as u64));
        let current_table: &mut crosshw::memory::paging::PageTable =
            unsafe { &mut *(ptr as *mut crosshw::memory::paging::PageTable) };
        let indices = PageHierarchy::vaddr_into_indices(kernel_offset + (0x1000 * i as u64));
        let pg = &current_table[indices.3];
        let pgptr = pg.address() as *mut u8;
        for j in 0..4095 {
            if i == kpagecount - 1 && j > kfinalpageend - 1 {
                break;
            }
            unsafe { *(pgptr.wrapping_add(j)) = kernel_file[i * 0x1000 + j] }
        }
    }
    let entryptr: *mut u8 = {
        let ptr = page_manager.get_table_at_vaddr(&mut fralloc, kernel_offset);
        let pt: &mut crosshw::memory::paging::PageTable =
            unsafe { &mut *(ptr as *mut crosshw::memory::paging::PageTable) };
        let indices = PageHierarchy::vaddr_into_indices(kernel_offset);
        let pg = &pt[indices.3];
        pg.address() as *mut u8
    };
    unsafe {
        page_manager.enable_paging();
    }
    let ptr = addr_of!(entryptr).wrapping_add(crosshw::memory::paging::OFFSET as usize); //Compensates for enabling paging.
    let kernel: fn() -> ! = unsafe { core::mem::transmute(ptr) };
    kernel(); //Bootvoid -> Kernelspace
    system_table.boot_services().stall(999_000_000);
    Status::SUCCESS
}
//CrossHW doesn't import the uefi crate; so we're implementing the uefi-specific frame allocator here.
#[repr(C, align(4096))]
pub struct MemoryMapContainer {
    pub mmap: Vec<MemoryDescriptor>,
}
impl MemoryMapContainer {
    pub fn new(system_table: &mut SystemTable<Boot>) -> MemoryMapContainer {
        let mut buf = [0u8; 8192];
        /*
            Box<[u8]> = {
            let tmp: Vec<_> =
                vec![0u8; system_table.boot_services().memory_map_size().map_size + 10];

            tmp.into_boxed_slice()
        };
        */
        let mm = system_table.boot_services().memory_map(&mut buf);
        if mm.is_err() {
            println!("{:#?}", mm.err());
        }
        let mut mmap = MemoryMap::from_raw(
            &mut buf,
            system_table.boot_services().memory_map_size().entry_size,
        )
        .entries()
        .copied()
        .collect::<Vec<_>>();
        MemoryMapContainer { mmap: mmap }
    }
}

pub struct UEFIFrameAlloc {
    pages: FreeList,
    physpagecount: usize,
}
impl UEFIFrameAlloc {
    pub fn init(&mut self) {
        let mut system_table = system_table();
        let memmap = MemoryMapContainer::new(&mut system_table);
        for i in memmap.mmap.iter() {
            self.physpagecount += i.page_count as usize;
            if i.ty == MemoryType::CONVENTIONAL {
                for page in 1..i.page_count {
                    let addr = i.phys_start + (4096 * (page - 1)) as u64;
                    self.pages.push(addr);
                }
            }
        }
    }
}
#[cfg(target_arch = "x86_64")]
unsafe impl FrameAllocator<Size4KiB> for UEFIFrameAlloc {
    fn allocate_frame(&mut self) -> Option<PhysFrame<Size4KiB>> {
        let ret =
            PhysFrame::<Size4KiB>::from_start_address(PhysAddr::new_truncate(self.pages.pop()));
        loop {}
        if ret.is_ok() {
            Some(ret.unwrap())
        } else {
            None
        }
    }
}
#[derive(Copy, Clone)]
#[repr(C, align(4096))]
pub struct FreeListNode {
    //Occupies a page, but gives access to 2040 KiB of memory.
    this: [u64; 510],
    next: u64,
    used: u64,
}
impl FreeListNode {
    pub unsafe fn from_ptr<'a>(ptr: u64) -> FreeListNode {
        unsafe { *((ptr as *mut u8) as *mut FreeListNode) }
    }
}
pub struct FreeList(FreeListNode);
impl FreeList {
    pub fn new() -> FreeList {
        let node = FreeListNode {
            this: [0u64; 510],
            next: 0,
            used: 0,
        };
        FreeList(node)
    }
    pub fn last(&self, penultimate: bool) -> FreeListNode {
        let mut node = self.0;
        let mut prevptr = 0u64;
        loop {
            if node.next == 0 {
                if penultimate {
                    break node;
                } else {
                    unsafe { break FreeListNode::from_ptr(prevptr) }
                }
            }
            prevptr = addr_of_mut!(node) as u64;
            node = unsafe { FreeListNode::from_ptr(node.next) }
        }
    }
    pub fn pop(&mut self) -> u64 {
        let mut last = self.last(false);
        let mut penultimate = self.last(true);
        println!("{:#?}", last.used);
        loop {}
        let ret = last.this[last.used as usize];
        let ret = if last.used >= 1 {
            last.used -= 1;
            last.this[(last.used + 1) as usize]
        } else {
            let rt = penultimate.next;
            penultimate.next = 0;
            rt
        };
        ret
    }
    pub fn push(&mut self, addr: u64) -> Option<()> {
        let mut last = self.last(false);
        if last.used == 510 && last.next == 0 {
            last.next = addr;
        } else if last.used <= 509 {
            last.this[last.used as usize] = addr;
        } else {
            return None;
        }
        Some(())
    }
}
