# LorentzGasSimulation
Simulation written for my master thesis for the examination of transport properties of a 2-dimensional Lorentz gas of charged particles under the influence of a magnetic field. For a summary of the setting and the results, go to
https://journals.aps.org/prb/abstract/10.1103/PhysRevB.102.081302

Compile with "make magneto".
Run with "./magneto.x t N R S" with t as lifetime of particles, N number of particles, R gyration radius, and S random seed.
Output: 7 csv files with two columns of double values. First column is time, second column gives one of 7 transport quantities:
mean-squared displacement (msd), msd in x-direction (msd_x), msd in y-direction (msd_y), velocity autocorrelation functions (acf_xx, acf_yy, acf_xy, acf_yx).
