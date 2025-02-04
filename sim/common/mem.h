#pragma once

#include <cstdint>
#include <vector>
#include <unordered_map>
#include <cstdint>

namespace vortex {
struct BadAddress {};

class MemDevice {
public:
  virtual ~MemDevice() {}
  virtual uint64_t size() const = 0;
  virtual void read(void *data, uint64_t addr, uint64_t size) = 0;
  virtual void write(const void *data, uint64_t addr, uint64_t size) = 0;
};

///////////////////////////////////////////////////////////////////////////////

class RamMemDevice : public MemDevice {
public:
  RamMemDevice(uint64_t size, uint32_t wordSize);
  RamMemDevice(const char *filename, uint32_t wordSize);
  ~RamMemDevice() {}

  void read(void *data, uint64_t addr, uint64_t size) override;  
  void write(const void *data, uint64_t addr, uint64_t size) override;

  virtual uint64_t size() const {
    return contents_.size();
  };

protected:
  std::vector<uint8_t> contents_;
  uint32_t wordSize_;
};

///////////////////////////////////////////////////////////////////////////////

class RomMemDevice : public RamMemDevice {
public:
  RomMemDevice(const char *filename, uint32_t wordSize)
    : RamMemDevice(filename, wordSize) 
  {}

  RomMemDevice(uint64_t size, uint32_t wordSize)
    : RamMemDevice(size, wordSize) 
  {}
  
  ~RomMemDevice();

  void write(const void *data, uint64_t addr, uint64_t size) override;
};

///////////////////////////////////////////////////////////////////////////////

class MemoryUnit {
public:
  
  struct PageFault {
    PageFault(uint64_t a, bool nf)
      : faultAddr(a)
      , notFound(nf) 
    {}
    uint64_t faultAddr;
    bool notFound;
  };

  MemoryUnit(uint64_t pageSize, uint64_t addrBytes, bool disableVm = false);

  void attach(MemDevice &m, uint64_t start, uint64_t end);

  void read(void *data, uint64_t addr, uint64_t size, uint64_t ptbr, uint32_t wid);  
  void write(const void *data, uint64_t addr, uint64_t size, uint64_t ptbr, uint32_t wid);

  void tlbFlush() {
    tlb_.clear();
  }
private:

  class ADecoder {
  public:
    ADecoder() {}
    
    void read(void *data, uint64_t addr, uint64_t size);
    void write(const void *data, uint64_t addr, uint64_t size);
    
    void map(uint64_t start, uint64_t end, MemDevice &md);

  private:

    struct mem_accessor_t {
      MemDevice* md;
      uint64_t addr;
    };
    
    struct entry_t {
      MemDevice *md;
      uint64_t      start;
      uint64_t      end;        
    };

    bool lookup(uint64_t a, uint32_t wordSize, mem_accessor_t*);

    std::vector<entry_t> entries_;
  };

  struct TLBEntry {
    TLBEntry() {}
    TLBEntry(uint32_t pfn, uint32_t flags)
      : pfn(pfn)
      , flags(flags) 
    {}
    uint32_t pfn;
    uint32_t flags;
  };

  MemoryUnit::TLBEntry tlbAdd(uint64_t virt, uint64_t phys, uint32_t flags);
  void tlbRm(uint64_t va);

  struct PTEntry{
    PTEntry() {};
    PTEntry(uint8_t valid, uint64_t pfn):
      valid(valid), pfn(pfn) {}

    uint8_t valid;
    uint64_t pfn;
  };

  TLBEntry handlePageFault(uint64_t vAddr, uint32_t flagMask, uint64_t ptbr);
  TLBEntry tlbLookup(uint64_t vAddr, uint32_t flagMask, uint64_t ptbr);

  std::unordered_map<uint64_t, TLBEntry> tlb_;
  uint64_t pageSize_;
  uint64_t addrBytes_;
  ADecoder decoder_;  
  bool disableVM_;
};

///////////////////////////////////////////////////////////////////////////////

class RAM : public MemDevice {
public:
  
  RAM(uint32_t page_size);
  ~RAM();

  void clear();

  uint64_t size() const override;

  void read(void *data, uint64_t addr, uint64_t size) override;  
  void write(const void *data, uint64_t addr, uint64_t size) override;

  uint64_t loadBinImage(const char* filename, uint64_t destination, uint64_t addrBytes);
  uint64_t loadHexImage(const char* filename, uint64_t addrBytes);

  uint8_t& operator[](uint64_t address) {
    return *this->get(address);
  }

  const uint8_t& operator[](uint64_t address) const {
    return *this->get(address);
  }

    // make page method

  // allocate physical frame table
  // page of physical frame entry structs -> just tracts which pages are being used
  // do not even need a physical frame table

  // really just need a page table at page (1)
  // we can put a dummy frame table at page (0)

  
private:

  uint8_t *get(uint64_t address) const;

  uint64_t size_;
  uint32_t page_bits_;
  mutable std::unordered_map<uint64_t, uint8_t*> pages_;
  mutable uint8_t* last_page_;
  mutable uint64_t last_page_index_;
};

struct FTEntry{
private:
  FTEntry() {};
  FTEntry(uint8_t protected_, uint8_t mapped, uint8_t referenced):
    protected_(protected_), mapped(mapped), referenced(referenced){};

  uint8_t protected_;          /* 1 if the frame holds a page table and is
                                  immune from eviction, 0 otherwise */
  /* -- Used for data pages -- */
  uint8_t mapped;             /* 1 if the frame is mapped, 0
                                  otherwise */
  uint8_t referenced;         /* 1 if the entry has been recently
                                  used, 0 otherwise */
  friend class MemoryUnit;
  friend class RAM;
};

} // namespace vortex
