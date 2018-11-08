```
/*******************************************************************************
*                           Balancebot Template Code
*                           pgaskell@umich.edu
*       
*******************************************************************************/

bin/			             : Binaries folder
balanceebot/balanceebot.c/.h : Main setup and threads
test_motors/test_motors.c/.h : Program to test motor implementation
common/mb_controller.c/.h    : Contoller for manual and autonomous nav
common/mb_defs.h             : Define hardware config
common/mb_motors.c/.h        : Motor functions to be used by balancebot
common/mb_odometry.c/.h	     : Odometry functions
```

## Sections to split up
- balance controller (section 2) - Phoneix
- odometry and motion control (section 3) - Johnson (will provide limited help for other sections)
- path planning and following (section 4) -

## How to automatically sync code from computer to Beaglebone:
1. (on computer) ssh-keygen to default `id_rsa` and `id_rsa.pub`
2. (on bbg) copy content of computer's `id_rsa.pub` to bbg's `~/.ssh/authorized_keys` (create file/directory if necessary)
3. (on computer) `python -m --user pip install watchdog`
4. download script to computer: https://gist.github.com/LemonPi/49e749489a46186665ea4a5e5e0819c1
5. run script passing in path to directory to sync `python watch_sync.py ubalance/`
    - if you are connecting via wifi, use `python watch_sync.py ubalance/ -w`
    - note that this should be run when you're cd'ed to `~/Documents`
## Determining motor parameters
- in steady state V = Ri + Kw
    - start measuring after spinning for 2s to ensure steady state
    - measure V across motors
    - measure i using current sense
    - measure w using encoder
    - get multiple data points and estimate R and K using linear regression
        - V = [i w][R K]'
        - [V; ...; V_N] = [i_1 w_1; ...; i_N w_N][R K]'
            - V = Ix
        - [R K]' = pseudoInv(I)V
- steady state no load speed ω_NL = KV/(K^2 + Rb)
    - estimate b = (KV/(ω_NL) - K^2) / R
    - ignore c for modelling
- determine friction coefficients b ,c
    - τ = Ki − bω − c sgn(ω)
    - ignoring c
    - set τ=0, then ωNL =KV/(K^2 + Rb)
    - b = (KV/ωNL - K^2)/R
- determine the Inertia of the motor armature, shaft and gearbox IM
    - Inertia = acceleration torque / acceleration
    - Apply a certain current to the motor
    - plot speed versus time, to get the acceleration
    - with an initial velocity, set duty to 0 to get
        - 0 = Jwdot + bω

Motor coil Resistance R   (5.4  5.6) +/- 0.1 Ohm ,makred on the arcrylic board
No Load Speed ⍵NL   measure the speed from encoder
Motor Constant K    V = Ri + Kω
Stall Torque 𝜏S     τ = Ki
Friction Coefficients (b & c or just b)   ωNL =KV/(K2 + Rb)
Inertia of the motor armature, shaft and gearbox IM

Reference from manufactuer's specs:
Gear ratio: 20.4:1
No-load speed @ 12V:    370 rpm
No-load current @ 12V:  200 mA
Stall current @ 12V:    2100 mA
Stall torque @ 12V: 42 oz·in
No-load speed @ 6V: 185 rpm1
Stall current @ 6V: 1050 mA1
Stall torque @ 6V:  21 oz·in1

## Determined parameters
Left K: 0.261708 +/- 0.044643
Right K: 0.282025 +/- 0.068916

Left w_NL: 44.193104 +/- 0.044183
Right w_NL: 40.627945 +/- 0.046306

Left b: 0.000114 +/- 0.002513
Right b: 0.000263 +/- 0.004255

Left J: 0.000046 +/- 0.000699
Right J: 0.000114 +/- 0.002082

## Determining odometry parameters
- attach castor to enable stable driving
- manually push robot along straight line of known length to determine wheel diameters
    - record the number of ticks per distance and divide to get ticks per m
- after outer loop controller is ready have robot turn in place to determine baseline
    - adjust baseline until its perceived full turn corresponds to an actual full turn