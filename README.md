# STM32F767ZI Timer Input Capture Encoder Mode

This project is dedicated on building an encoder reader to capture the pulse signals generated from a stepper motor's build-in encoder.

The encoder provides A, B, and Z phase signals. Both of A and B phases produce pulses at the frequency of 500 pulses/revolution. In terms of Z phase, the encoder produces sigle pulse at Z phase channel for each complete revolution.

In this project, we only take into account A and B signals. And set up a counter overflow interrupt to count the revolutions, instead of using Z signal or periodically reading current value of the counter to determine the revolutions.
