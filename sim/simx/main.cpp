#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>
#include <fstream>
#include <stdlib.h>
#include <sys/stat.h>
#include "processor.h"
#include "archdef.h"
#include "mem.h"
#include "constants.h"
#include <util.h>
#include "args.h"
#include "core.h"

using namespace vortex;

int main(int argc, char **argv) {
  int exitcode = 0;

  std::string imgFileName, imgFileName2;
  int num_cores(NUM_CORES * NUM_CLUSTERS);
  int num_warps(NUM_WARPS);
  int num_threads(NUM_THREADS);  
  bool showHelp(false);
  bool showStats(false);
  bool riscv_test(false);

  // parse the command line arguments
  CommandLineArgFlag fh("-h", "--help", "show command line options", showHelp);
  CommandLineArgSetter<std::string> fi("-i", "--image", "program binary", imgFileName);
  CommandLineArgSetter<std::string> fi2("-i2", "--image2", "program binary", imgFileName2);
  CommandLineArgSetter<int> fc("-c", "--cores", "number of cores", num_cores);
  CommandLineArgSetter<int> fw("-w", "--warps", "number  of warps", num_warps);
  CommandLineArgSetter<int> ft("-t", "--threads", "number of threads", num_threads);
  CommandLineArgFlag fr("-r", "--riscv", "enable riscv tests", riscv_test);
  CommandLineArgFlag fs("-s", "--stats", "show stats", showStats);

  CommandLineArg::readArgs(argc - 1, argv + 1);

  if (showHelp || imgFileName.empty()) {
    std::cout << "Vortex emulator command line arguments:\n"
                 "  -i, --image <filename> Program RAM image\n"
                 "  -i2, --image2 <filename> Second program RAM image\n"
                 "  -c, --cores <num> Number of cores\n"
                 "  -w, --warps <num> Number of warps\n"
                 "  -t, --threads <num> Number of threads\n"
                 "  -r, --riscv riscv test\n"
                 "  -s, --stats Print stats on exit.\n";
    return 0;
  }

  std::cout << "Running " << imgFileName << "..." << std::endl;
  
  {
    // create memory module
    RAM ram(RAM_PAGE_SIZE);

    std::vector<uint64_t> ptbrs;


    // load program
    {
      uint64_t ptbr;
      std::string program_ext(fileExtension(imgFileName.c_str()));
      if (program_ext == "bin") {
        ptbr = ram.loadBinImage(imgFileName.c_str(), STARTUP_ADDR,ArchDef(0,0,0,ptbrs).wsize());
      } else if (program_ext == "hex") {
        // return PTBR for this process on image load
        // starting address PC=0x80000000
        ptbr = ram.loadHexImage(imgFileName.c_str(),ArchDef(0,0,0,ptbrs).wsize());
      } else {
        std::cout << "*** error: only *.bin or *.hex images supported." << std::endl;
        return -1;
      }
      ptbrs.push_back(ptbr);
    }



    // load program 2
    {
      uint64_t ptbr2;
      if (!imgFileName2.empty()){
        std::cout << " Also running " << imgFileName2 << "..." << std::endl;
        std::string program_ext(fileExtension(imgFileName2.c_str()));
        if (program_ext == "bin") {
          ptbr2 = ram.loadBinImage(imgFileName2.c_str(), STARTUP_ADDR,ArchDef(0,0,0,ptbrs).wsize());
        } else if (program_ext == "hex") {
          // return PTBR for this process on image load
          // starting address PC=0x80000000
          ptbr2 = ram.loadHexImage(imgFileName2.c_str(),ArchDef(0,0,0,ptbrs).wsize());
        } else {
          std::cout << "*** error: only *.bin or *.hex images supported." << std::endl;
          return -1;
        }
        ptbrs.push_back(ptbr2);
        if(num_warps % 2 != 0){
          std::cout << "*** error: num warps must be multiple of 2 for 2 programs." << std::endl;
          return -1;
        }
      }
    }


    // create processor configuation
    ArchDef arch(num_cores, num_warps, num_threads, ptbrs);


    // create processor
    Processor processor(arch);
  
    // attach memory module
    processor.attach_ram(&ram);   

    // run simulation
    exitcode = processor.run();

  } 

  if (riscv_test) {
    if (1 == exitcode) {
      std::cout << "Passed." << std::endl;
      exitcode = 0;
    } else {
      std::cout << "Failed." << std::endl;
    }
  } else {
    if (exitcode != 0) {
      std::cout << "*** error: exitcode=" << exitcode << std::endl;
    }
  }  

  return exitcode;
}
