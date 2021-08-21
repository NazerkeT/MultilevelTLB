# Multilevel TLB

Multilevel Translation Lookaside Buffer (TLB) implementation workspace for (CVA6) Ariane Core. <br />
More is to come, keep tuned! ;)

General tasks:
*   L2 TLB design (done);
*   Integrating L2 TLB to MMU (done);
*   Module and MMU level simulations (done -> bugs fixed and stable results);
*   Updating PTW (done);
*   Updating Load Store Unit (LSU) (done);
*   Updating Instruction Cache Interface (no need);
*   Other core level changes (no need); 
*   Testbenching (partially completed);
*   Test in OpenPiton (done -> successful compilation of the "Hello, world!" program);

Future taks to be completed:
*   FSM based implementation of L2 TLB flush logic;
*   SRAM implementation of the L2 TLB memory;
*   Testing virtual memory performance with targeted benchmarks;
*   Contribution to the Ariane;

Useful links:
*   GitHub page of the core: https://github.com/openhwgroup/cva6
*   Documentation of the core: https://cva6.readthedocs.io/en/latest/
*   The RISC-V Instruction Set Manual, Volume II: Privileged Architecture, 2019
*   A. Bhattacharjee, D. Lustig, 2017. _Architectural and Operating System Support for Virtual Memory, Synthesis Lectures on Computer Architecture_, https://doi.org/10.2200/S00795ED1V01Y201708CAC042
*   A. Silberschatz, P. B. Galvin, G. Gagne, 2013. _Operating System Concepts_, 9th Edition.

