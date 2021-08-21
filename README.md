
<img src="https://developers.google.com/open-source/gsoc/resources/downloads/GSoC-logo-horizontal.svg" width=700  />

<p float="left">
  <img src="https://www.fossi-foundation.org/assets/fossi_logo_large.png" width=95 /> 
  &nbsp;&nbsp;
  <img src="http://parallel.princeton.edu/openpiton/images/openpiton_ariane_logo.png" width=700/> 
</p>

# Multilevel TLB
It is a repository dedicated to the summer GSoC'21 project - Multilevel Translation Lookaside Buffer (TLB) for 64-bit RISC-V (CVA6) Ariane Core. An updated TLB structure contains existing L1 TLBs for instruction and data interfaces, also a new shared L2 TLB for both request types. Upon translation request and hit in one of the TLBs, a result is available either in the same or in the third cycle at the longest. It will depend on the size of the requested page, which varies between 4KB, 2MB and 1GB. L2 TLB is considerably larger than L1 TLB, and here to compensate for the translation lookup miss rate at L1 TLB with increased capacity at L2 TLB. If a miss occurs in one of the L1 TLBs, L2 TLB will be looked up. If it also misses, then Page Table Walk (PTW) is executed. It, in its turn, increases the overall performance of the core. 

A multilevel TLB based virtual memory is the industry standard for many application class CPUs, so accordingly was a compelling project to discover :) Nevertheless, future tasks are equally significant milestones, as well, and are to be continued apart from the GSoC scope.  <br />

General tasks:
*   L2 TLB design (done -> multi-cycle, size-configurable hash-rehash structure);
*   Integrating L2 TLB to MMU (done);
*   Module and MMU level simulations (done -> bugs fixed and stable results);
*   Updating PTW (done);
*   Updating Load Store Unit (LSU) (done);
*   Updating Instruction Cache Interface (no need);
*   Other core level changes (no need); 
*   Testbenching (partially completed) ;
*   Test in OpenPiton (done -> successful compilation of the "Hello, world!" program);

Future tasks to be completed:
*   FSM based implementation of the L2 TLB flush logic;
*   SRAM implementation of the L2 TLB memory;
*   Testing virtual memory performance with targeted benchmarks;
*   Contribution to the Ariane;

Useful links:
*   GitHub page of the core: https://github.com/openhwgroup/cva6
*   Documentation of the core: https://cva6.readthedocs.io/en/latest/
*   The RISC-V Instruction Set Manual, Volume II: Privileged Architecture, 2019
*   A. Bhattacharjee, D. Lustig, 2017. _Architectural and Operating System Support for Virtual Memory, Synthesis Lectures on Computer Architecture_, https://doi.org/10.2200/S00795ED1V01Y201708CAC042
*   A. Silberschatz, P. B. Galvin, G. Gagne, 2013. _Operating System Concepts_, 9th Edition.

