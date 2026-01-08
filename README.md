This project implements four different pipelined processor models: non-forwarding, forwarding, always-taken, two-bit Dynamic Branch Prediction.

The file allmodule contains all modules required for implementation of the pipelined processor, including:
 - All module in the single-cycle processor (with necessary modifications).
 - Hazard Detection Unit.
 - Forwarding Unit.
 - Branch Target Buffer (BTB) for always-taken, Two-Bit Dynamic Branch Prediction.
 - Branch History Table (BHT) for two-bit Dynamic Branch Prediction.

The file pipelined contains the top-level module, which instantiates and connects all the required modules to form the complete pipelined processor.
