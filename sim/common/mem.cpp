#include "mem.h"
#include <vector>
#include <iostream>
#include <fstream>
#include <assert.h>
#include "util.h"

using namespace vortex;


#define DEBUG(args) 
#define DEBUG(args) args

RamMemDevice::RamMemDevice(const char *filename, uint32_t wordSize) 
  : wordSize_(wordSize) {
  std::ifstream input(filename);

  if (!input) {
    std::cout << "Error reading file \"" << filename << "\" into RamMemDevice.\n";
    std::abort();
  }

  do {
    contents_.push_back(input.get());
  } while (input);

  while (contents_.size() & (wordSize-1))
    contents_.push_back(0x00);
}

RamMemDevice::RamMemDevice(uint64_t size, uint32_t wordSize)
  : contents_(size) 
  , wordSize_(wordSize)
{}

void RamMemDevice::read(void *data, uint64_t addr, uint64_t size) {
  auto addr_end = addr + size;
  if ((addr & (wordSize_-1))
   || (addr_end & (wordSize_-1)) 
   || (addr_end <= contents_.size())) {
    std::cout << "lookup of 0x" << std::hex << (addr_end-1) << " failed.\n";
    throw BadAddress();
  }  
  
  const uint8_t *s = contents_.data() + addr;
  for (uint8_t *d = (uint8_t*)data, *de = d + size; d != de;) {
    *d++ = *s++;
  }
}

void RamMemDevice::write(const void *data, uint64_t addr, uint64_t size) {
  auto addr_end = addr + size;
  if ((addr & (wordSize_-1))
   || (addr_end & (wordSize_-1)) 
   || (addr_end <= contents_.size())) {
    std::cout << "lookup of 0x" << std::hex << (addr_end-1) << " failed.\n";
    throw BadAddress();
  }

  const uint8_t *s = (const uint8_t*)data;
  for (uint8_t *d = contents_.data() + addr, *de = d + size; d != de;) {
    *d++ = *s++;
  }
}

///////////////////////////////////////////////////////////////////////////////

void RomMemDevice::write(const void* /*data*/, uint64_t /*addr*/, uint64_t /*size*/) {
  std::cout << "attempt to write to ROM.\n";
  std::abort();
}

///////////////////////////////////////////////////////////////////////////////

bool MemoryUnit::ADecoder::lookup(uint64_t a, uint32_t wordSize, mem_accessor_t* ma) {
  uint64_t e = a + (wordSize - 1);
  assert(e >= a);
  for (auto iter = entries_.rbegin(), iterE = entries_.rend(); iter != iterE; ++iter) {
    if (a >= iter->start && e <= iter->end) {
      ma->md   = iter->md;
      ma->addr = a - iter->start;
      return true;
    }
  }
  return false;
}

void MemoryUnit::ADecoder::map(uint64_t a, uint64_t e, MemDevice &m) {
  assert(e >= a);
  entry_t entry{&m, a, e};
  entries_.emplace_back(entry);
}

void MemoryUnit::ADecoder::read(void *data, uint64_t addr, uint64_t size) {
  mem_accessor_t ma;
  if (!this->lookup(addr, size, &ma)) {
    std::cout << "lookup of 0x" << std::hex << addr << " failed.\n";
    throw BadAddress();
  }      
  ma.md->read(data, ma.addr, size);
}

void MemoryUnit::ADecoder::write(const void *data, uint64_t addr, uint64_t size) {
  mem_accessor_t ma;
  if (!this->lookup(addr, size, &ma)) {
    std::cout << "lookup of 0x" << std::hex << addr << " failed.\n";
    throw BadAddress();
  }
  ma.md->write(data, ma.addr, size);
}

///////////////////////////////////////////////////////////////////////////////

MemoryUnit::MemoryUnit(uint64_t pageSize, uint64_t addrBytes, bool disableVm)
  : pageSize_(pageSize)
  , addrBytes_(addrBytes)
  , disableVM_(disableVm) {
  // if (!disableVm) {
    // I don't think we need this dummy entry?
    // tlb_[0] = TLBEntry(0, 077);
  // }
}

void MemoryUnit::attach(MemDevice &m, uint64_t start, uint64_t end) {
  decoder_.map(start, end, m);
}

MemoryUnit::TLBEntry MemoryUnit::handlePageFault(uint64_t vAddr, uint32_t flagMask, uint64_t ptbr) {
  DEBUG(std::cout << "faulting" << std::endl;)
  uint64_t PTEaddr = ptbr * pageSize_ + (vAddr / pageSize_  - 0x80000) * sizeof(PTEntry);
  DEBUG(std::cout << ptbr << " " << std::hex << PTEaddr << std::endl;)
  uint8_t valid;
  decoder_.read(&valid, PTEaddr, 1);
  if (valid){
    DEBUG(std::cout << "valid" << std::endl;)
    uint64_t pfn;
    decoder_.read(&pfn, PTEaddr+8, sizeof(pfn));
    DEBUG(std::cout << pfn << std::endl;)
    return tlbAdd(vAddr, pfn, flagMask);
  }
  else {
    // we need a new page table entry
    uint64_t pfn = -1;
    uint8_t pfn_mapped;
    // fetch a new physical frame
    do{
      decoder_.read(&pfn_mapped, 1 + sizeof(FTEntry) * ++pfn, 1);
    }
    while(pfn_mapped == 1);
    DEBUG(std::cout << "pfn " << pfn << std::endl;)

    FTEntry page_table;
    page_table.protected_ = 1;
    page_table.mapped = 1;
    decoder_.write(&page_table, sizeof(FTEntry) * pfn, sizeof(FTEntry));

    uint8_t zero = 0;
    for (uint64_t i = 0; i < pageSize_; i++)
      decoder_.write(&zero, i + pfn * pageSize_, 1);

    

    PTEntry entry;
    entry.valid = 1;
    entry.pfn = pfn;
    DEBUG(std::cout << entry.pfn << std::endl;)
    decoder_.write(&entry, PTEaddr, sizeof(entry));
    return tlbAdd(vAddr, pfn, flagMask);
  }
}

MemoryUnit::TLBEntry MemoryUnit::tlbLookup(uint64_t vAddr, uint32_t flagMask, uint64_t ptbr) {
  // vAddr / pageSize => vpn
  DEBUG(std::cout << std::hex << vAddr / pageSize_ << std::endl;)
  auto iter = tlb_.find(vAddr / pageSize_);
  if (iter != tlb_.end()) {
    if (iter->second.flags & flagMask)
      return iter->second;
    else {
      //PAGE FAULT
      DEBUG(std::cout << "flags" << std::endl;)
      return handlePageFault(vAddr, flagMask, ptbr);
    }
  } else {
      DEBUG(std::cout << "miss" << std::endl;)
    // throw PageFault(vAddr, true);
      return handlePageFault(vAddr, flagMask, ptbr);
  }
}

void MemoryUnit::read(void *data, uint64_t addr, uint64_t size, bool sup, uint64_t ptbr) {
  DEBUG(std::cout << std::hex << addr << std::endl;)
  uint64_t pAddr;
  if (disableVM_) {
    pAddr = addr;
  } else {
    uint32_t flagMask = sup ? 8 : 1;
    TLBEntry t = this->tlbLookup(addr, flagMask, ptbr);
    //                          this is the offset portion
    pAddr = t.pfn * pageSize_ + addr % pageSize_;
    DEBUG(std::cout << std::hex << t.pfn << std::endl;)

  }
  DEBUG(std::cout << std::hex << pAddr << std::endl;)
  return decoder_.read(data, pAddr, size);
}

void MemoryUnit::write(const void *data, uint64_t addr, uint64_t size, bool sup, uint64_t ptbr) {
  DEBUG(std::cout << std::hex << addr << std::endl;)
  uint64_t pAddr;
  if (disableVM_) {
    pAddr = addr;
  } else {
    uint32_t flagMask = sup ? 8 : 1;
    TLBEntry t = tlbLookup(addr, flagMask, ptbr);
    pAddr = t.pfn * pageSize_ + addr % pageSize_;
  }
  DEBUG(std::cout << std::hex << pAddr << std::endl;)
  decoder_.write(data, pAddr, size);
}

MemoryUnit::TLBEntry MemoryUnit::tlbAdd(uint64_t virt, uint64_t pfn, uint32_t flags) {
  DEBUG(std::cout << virt / pageSize_ << std::endl;)
  tlb_[virt / pageSize_] = TLBEntry(pfn, flags);
  return tlb_[virt / pageSize_];
}

void MemoryUnit::tlbRm(uint64_t va) {
  if (tlb_.find(va / pageSize_) != tlb_.end())
    tlb_.erase(tlb_.find(va / pageSize_));
}

///////////////////////////////////////////////////////////////////////////////

RAM::RAM(uint32_t page_size) 
  : size_(0)
  , page_bits_(log2ceil(page_size))
  , last_page_(nullptr)
  , last_page_index_(0) {    
   assert(ispow2(page_size));
}

RAM::~RAM() {
  this->clear();
}

void RAM::clear() {
  for (auto& page : pages_) {
    delete[] page.second;
  }
}

uint64_t RAM::size() const {
  return uint64_t(pages_.size()) << page_bits_;
}

uint8_t *RAM::get(uint64_t address) const {
  uint32_t page_size   = 1 << page_bits_;  
  uint32_t page_offset = address & (page_size - 1);
  // page number
  uint64_t page_index  = address >> page_bits_;

  uint8_t* page;
  if (last_page_ && last_page_index_ == page_index) {
    page = last_page_;
  } else {
    auto it = pages_.find(page_index);
    if (it != pages_.end()) {
      page = it->second;
    } else {
      // makes a new page and adds it to the RAM
      uint8_t *ptr = new uint8_t[page_size];
      DEBUG(std::cout << ptr << std::endl;)
      // set uninitialized data to "baadf00d"
      for (uint32_t i = 0; i < page_size; ++i) {
        ptr[i] = (0xbaadf00d >> ((i & 0x3) * 8)) & 0xff;
      }
      pages_.emplace(page_index, ptr);
      page = ptr;
      DEBUG(std::cout << page << std::endl;)
    }
    last_page_ = page;
    last_page_index_ = page_index;
  }

  return page + page_offset;
}

void RAM::read(void *data, uint64_t addr, uint64_t size) {
  uint8_t* d = (uint8_t*)data;
  for (uint64_t i = 0; i < size; i++) {
    d[i] = *this->get(addr + i);
  }
}

void RAM::write(const void *data, uint64_t addr, uint64_t size) {
  const uint8_t* d = (const uint8_t*)data;
  for (uint64_t i = 0; i < size; i++) {
    *this->get(addr + i) = d[i];
  }
}

uint64_t RAM::loadBinImage(const char* filename, uint64_t destination, uint64_t addrBytes) {
  std::ifstream ifs(filename);
  if (!ifs) {
    std::cout << "error: " << filename << " not found" << std::endl;
  }

  ifs.seekg(0, ifs.end);
  size_t size = ifs.tellg();
  std::vector<uint8_t> content(size);
  ifs.seekg(0, ifs.beg);
  ifs.read((char*)content.data(), size);

  this->clear();

  FTEntry frame_table;
  frame_table.protected_ = 1;
  frame_table.mapped = 1;
  for (int i = 0; i < 1 << page_bits_; i++)
    *this->get(i) =  0;
  this->write(&frame_table, 0, sizeof(FTEntry));

  // need to return the physical frame number corresponding to this program
  // just find the next unmapped pfn
  uint64_t pfn = -1;
  while(*this->get(1 + sizeof(FTEntry) * ++pfn) == 1 ){}

  FTEntry page_table;
  page_table.protected_ = 1;
  page_table.mapped = 1;
  for (int i = 0; i < 1 << page_bits_; i++)
    *this->get(i + pfn * (1 << page_bits_)) =  0;
  this->write(&page_table, sizeof(FTEntry) * pfn, sizeof(FTEntry));

  MemoryUnit mmu_(1 << page_bits_, addrBytes, false);

  mmu_.attach(*this, 0, 0xFFFFFFFF);

  mmu_.write(content.data(), destination, size, 0, pfn);

  return pfn;
}

uint64_t RAM::loadHexImage(const char* filename, uint64_t addrBytes) {
  auto hti = [&](char c)->uint32_t {
    if (c >= 'A' && c <= 'F')
      return c - 'A' + 10;
    if (c >= 'a' && c <= 'f')
      return c - 'a' + 10;
    return c - '0';
  };

  auto hToI = [&](const char *c, uint32_t size)->uint32_t {
    uint32_t value = 0;
    for (uint32_t i = 0; i < size; i++) {
      value += hti(c[i]) << ((size - i - 1) * 4);
    }
    return value;
  };

  std::ifstream ifs(filename);
  if (!ifs) {
    std::cout << "error: " << filename << " not found" << std::endl;
  }

  ifs.seekg(0, ifs.end);
  size_t size = ifs.tellg();
  std::vector<char> content(size);
  ifs.seekg(0, ifs.beg);
  ifs.read(content.data(), size);

  uint32_t offset = 0;
  char *line = content.data();

  this->clear();

  // this is where we need to maintain the PFN
  // put the frame table at address 0
  // FTEntry* frame_table = reinterpret_cast<FTEntry*>(this->get(0));

  FTEntry frame_table;
  frame_table.protected_ = 1;
  frame_table.mapped = 1;
  for (int i = 0; i < 1 << page_bits_; i++)
    *this->get(i) =  0;
  this->write(&frame_table, 0, sizeof(FTEntry));

  // need to return the physical frame number corresponding to this program
  // just find the next unmapped pfn
  uint64_t pfn = -1;
  while(*this->get(1 + sizeof(FTEntry) * ++pfn) == 1 ){}

  FTEntry page_table;
  page_table.protected_ = 1;
  page_table.mapped = 1;
  for (int i = 0; i < 1 << page_bits_; i++)
    *this->get(i + pfn * (1 << page_bits_)) =  0;
  this->write(&page_table, sizeof(FTEntry) * pfn, sizeof(FTEntry));

  MemoryUnit mmu_(1 << page_bits_, addrBytes, false);

  mmu_.attach(*this, 0, 0xFFFFFFFF);

  while (true) {
    if (line[0] == ':') {
      uint32_t byteCount = hToI(line + 1, 2);
      uint32_t nextAddr = hToI(line + 3, 4) + offset;
      uint32_t key = hToI(line + 7, 2);
      switch (key) {
      case 0:
        for (uint32_t i = 0; i < byteCount; i++) {
          uint32_t addr  = nextAddr + i;
          uint32_t value = hToI(line + 9 + i * 2, 2);
          // this->write(&value, addr, sizeof(value));
          // need to make this addr virtualized
          DEBUG(std::cout << std::hex << addr << std::endl;)
          mmu_.write(&value, addr, 1, 0, pfn);
          // *this->get(addr) = value;
        }
        break;
      case 2:
        offset = hToI(line + 9, 4) << 4;
        break;
      case 4:
        offset = hToI(line + 9, 4) << 16;
        break;
      default:
        break;
      }
    }
    while (*line != '\n' && size != 0) {
      ++line;
      --size;
    }
    if (size <= 1)
      break;
    ++line;
    --size;
  }
  
  return pfn;
}