# CCL Toolbox

There are some simple patterns that you can use with the CCL/Logic library to generate very widly applicable effects:

## Reordering inputs
Say
TRUTH = 0bHGFEDCBA when IN0 is α, IN1 is β and IN2 is γ
TRUTH = 0bHDFBGCEA when IN0 is γ, IN1 is β and IN2 is α - D and G, B and E swap
TRUTH = 0bHFGEDBCA when IN0 is α, IN1 is γ and IN2 is β - G and F, B and C swap
TRUTH = 0bHGDCFEBA when IN0 is γ, IN1 is α and IN2 is β - F and D, E and C swap
TRUTH = 0bHFDBGECA when IN0 is β, IN1 is γ and IN2 is α - F->G->D->F rotate, and B->E->C->B rotate.
TRUTH = 0bHDGCFBEA when IN0 is β, IN1 is α and IN2 is γ - F->D->G->F rotate, and B->C->E->B rotate.

the highest and lowest bits do not change when reordering the inputs.

Note also that not all truthtables have six distinct permutations that can be made by reordering the inputs. Ignoring the MSB and LSB, which are unchanged by input swapping - each of these thus has 4 variants with each possible permutation of those bits), of the 64 possible permutations:
4 do not change no matter how you switch around the inputs (assuming we've bitwise and'ed truth with 0b01111110 and have not shifted it in any way):
0x00. 0x16, 0x68, 0x7E

A significant number of options come in sets of three, unsurprising (again, MSB and LSB omitted)
* 0x02, 0x04, 0x10
* 0x06, 0x12, 0x14
* 0x08, 0x20, 0x40
* 0x1E, 0x36, 0x56
* 0x28, 0x48, 0x60
* 0x3E, 0x5E, 0x76
* 0x6A, 0x6C, 0x78
* 0x6E, 0x7A, 0x7C

Finally, the remaining ones form 4 groups of 8:
* 0x0A,0x0C,0x18,0x22,0x24,0x30,0x42,0x50
* 0x1A,0x1C,0x26,0x32,0x34,0x46,0x52,0x54
* 0x2A,0x2C,0x4A,0x4C,0x58,0x62,0x64,0x70
* 0x2E,0x3A,0x3C,0x4E,0x5C,0x66,0x72,0x74

Reordering the inputs, according to my calculations, can rearrange any of these into 3 to 5 others on that list, and not into any others. I have not yet been able to rationalize this observation.

## Examples

In the below examples X, Y, Z and 2 are used to refer to inputs. X, Y, and Z can refer to any input, but for the purposes of the LUT presented, X = 0, Y = 1, Z = 2

Input 2, since it can be used as a clock, may be specifically required.

## Feedback
The "Feedback" channel is the **output of the sequencer if used, and the even LUT in that pair if not**,

### But I need feedback on an ODD LUT
"Ya can't get there from here". Well, you can, but it'll cost you. The price is one event channel: Use the output as a generator, and set one of the event inputs of the CCL block to that channel

## A bit more on clocks and timing
The clocks have some counterintuitive behavior. First off, what do they and do they not effect? The clocks are used by:
* The Filter or Synchronizer, if enabled.
* The sequencer, if it is configured as a flip-flop.
* The Edge Detector
It does not govern the speed of the response otherwise - the CCL reacts asynchronously (it is not clocked, and happens as fast as the silicon can manage) unless the edge detector, synchronizer, filter, or sequencer are enabled.

### The edge detector
Sometimes you need a pulse when all you have is a level. This gets you there. The clock is involved because the resulting pulse is 1 CCL clock long (occasionally this is not long enough, since the CCL clock can be faster than the system clock, particularly on the EB, where you can clock the CCL from the PLL, or you may be using a very slow clock, and it could be troublesome how long it is.

### The ~programmable delay~ synchronizer/filter
This is one of the really cool, repurposable features. The intended use is that you can use the synchronizer to take a 2 clock cycle delay to ensure clean transitions and prevent glitches, with the filter meant to provide a means of cleanly handling more substantial noise by requiring that the signal be unchanged for 4 CCL clocks before outputting it. One nuisance is that according to the datasheet, one of these settings must be enabled to use the synchronizer.

### The sequencer
The sequencer, in some modes, also uses the clock. It takes the clock from the EVEN LUT when acting as a flip-flop (but not when acting as a latch - the latch isn't clocked).

### The signal proceeds through a logic block's subcomponents in an order
It's important to understand the signal path.
1. The LUT - asynchronously monitors it's inputs and outputs a value based on the truth table.
2. The Filter/Synchronizer - Takes input from the LUT. Either does nothing, performs a 2 clock synchronization, or performs synchronization, then feeds that through 2 additional flipflops, and compares the output of the last one with the sync output, and only changes if the two match, indicating a signal stable for more than 2 clocks. This introduces a delay of 2 or 4 clocks
3. The edgedetector, if enabled, takes the output from the filter/synchronizer, and reacts to edges in that (note that per datasheet, you need to have one of those two enabled - unclear if that's "Or else" or "Because we put an interlock to prevent you from doing that"). That pulse starts as soon as the signal gets in, and lasts one clock.
4. Finally, if the sequencer is enabled, it overrides the output of the even LUT.
5. This signal - the sequencer if enabled / Even LUT's output if not - is what feeds both the pin (if enabled) and the signal that is provided as feedback (if used).

Thus - the LUT comes first. It's output then may or may not be filtered/synchronized and if it is, may be edge-detected. If we imagine a LUT taking say, the OR of 2 inputs, and put a filter on it, and each of those inputs is (for simplicity) changing according to the same clock as the LUT. If the two inputs are, repeating: HLL and LHL, the output from the first stage will be HHL. The filter will then turn that into *a continuous high level*.

## PWM is magic
Unlike the event channels, where you get a single clock long pulse from a compare match or overflow, the inputs to logic blocks are the level of the output compare! That means that you can remap pins that don't exist on your part but would have PWM if they did to the LUT output pin.

### Long-armed PWM
Reach out and use a timer on a distant pin instead of the (otherwise occupied or non-existent) one it would normally get.

INSEL:
```text
X: TCA WO0, TCB0 or TCD WOA
Y: masked
Z: masked
or
X: masked
Y: TCA WO1, TCB1 or TCD WOB
Z: masked
or
X: masked
Y: masked
Z: TCA WO2, TCB2 or TCD WOC (which is WOA)
```

LUT - Only one input is used, so there will be 2 bits that matter and 6 that don't (and should be left 0 by convention), one will be 0 (the low bit - you want ) and the other 1. So for non-inverted PWM, this is just
* 0x02 (0b00000010)
* 0x04 (0b00000100)
* 0x10 (0b00010000)

Clock: N/A

Sync/Filter: Off

Note that TCA WO3-5, TCB3+, and TCD WOD are not available as inputs.

### Out-of-phase PWM
Problem: You have 3 output channels, each of which requires PWM at a different duty cycle (total not exceeding 100%), but they control large loads, and your power supply can only power one at a time.

If you use a TCA (remembering to call the takeover function), you can run it in in SINGLE mode, getting 16 bits of resolution.

If you then used it normally, the three duty cycles would be written to CMP0, CMP1, and CMP2, but they would all turn on at once, that's not okay. Instead, you could set them as:
* CMP0 = DS0 - 1 -> Output on the pin by setting pin output and writing 1 to the CMP0 bit
* CMP1 = DS0 + DS1 -1 -> Output through the CCL
* CMP2 = PER - DS3 -> Output on the pin by setting pin output, setting INVEN (inverting the pin), writing 1 to the CMP2 bit.

CMP0 < CMP1 - Ensured by above, provided that:

DS0 + DS1 <= PER - the duty cycles of the first two pins must not exceed 100%

To meet the specific specification (one at a time only) we would need to be sure that DS0 + DS1 + DS2 <= PER, but not all applications require that of the final duty cycle (maybe there are only two big loads, and one light load... (possibly a cooling fan on the power supply?)

INSEL:
* X: WO0
* Y: WO1
* Z: masked

LUT:
* 000: 0 - During the time before CMP0 is reached, the second big load shouldn't be on.
* 001: 1 - Now CMP0 has been reached but CMP1 hasn't, so here is where we want the to turn this output, controlling the second load.
* 010: 0 - This is never reachable in practice if CMP1 > CMP0.
* 011: 0 - Once CMP1 and CMP0 have been passed, we turn off WO

Clock: N/A

Sync/Filter: Off

Yes, we could have done the third one with another CCL lut too - but why when there's a trick to do it without wasting a second LUT? Maybe we need the other LUTs.

### Modulated PWM
Like classic AVRs had on larger pincount devices. One PWM frequency should be significantly higher than the other if you're trying to modulate it, rather than measure the beat frequency or something, and they definitely should be at different frequencies, otherwise see the previous pattern.

INSEL:
* X: Timer PWM channel.
* Y: Second PWM channel.
* Z: masked

LUT: 0x08 (0b00001000, HIGH if both inputs are high)

Clock: N/A

Sync/Filter: Off

## Sequential logic with just one LUT
You can simulate some sequential logic units with just one LUT!

Enable the synchronizer to get the analogous flip-flop.

### S-R latch

INSEL:
* X: Feedback
* Y: Set (any input source)
* Z: Clear (any input source)

LUT:
* 000: 0
* 001: 1
* 010: 1
* 011: 1
* 100: 0
* 101: 0
* 110: Per application requirements - logic block is getting contradictroy signals
* 111: Per application requirements - logic block is getting contradictroy signals
Ergo: TRUTH = 0x0b??001110 = 0x07 (go low when told to go both directions), 0xC7 (go high when...) or 0x47 (don't change when...). Avoid 0x87 (Oscillate rapidly at an unpredictable speed when...)

Clock: N/A for latch, anything except IN2 as clock as demanded by application for flipflop.

Sync/Filter: Off for latch, on for flip-flop.

### D-Type latch

INSEL:
* X: Feedback
* Y: D (gated signal to latch)
* Z: G (Gate)

LUT:
* 000: 0
* 001: 1
* 010: 0
* 011: 1
* 100: 0
* 101: 0
* 110: 1
* 111: 1
Ergo: TRUTH = 0xCB

Clock: N/A for latch, anything except IN2 as clock as demanded by application for flipflop.

Sync/Filter: Off for latch, on for flip-flop.


### Pulse-stretcher
You often need pulses output on pins or to direct internal peripherals via the event system. Maybe you have an input that generates fast pulses, but you need a long enough pulse for a synchronous peripheral to react to, or maybe you'd like to generate a pulse from software, and then immediately write to another pin *during* the pulse, because that would make some bigbanged mess cleaner (I doubt it will, but feel free to try).

#### Even LUT
Start with the even LUT set to act as an S/R latch as described above.

INSEL:
* Set: Pulse input
* Reset: Link
* Feedback

LUT per above.

#### Odd LUT
The odd lut is a simple delay.

INSEL:
X: Feedback (from Even LUT)
Y: Masked
Z: Clock or Masked

LUT: 0x02

Clock: Anything that 2 or 4 ticks of will be a long enough pulse.

Sync/Filter: Sync to stretch any pulse on the input to 2 CLK<sub>odd</sub>, filter for 4 CLK<sub>odd</sub>.

Edge detector: No

This combination does the following:
1. Input signal sets the LUT acting as an SR latch. This generates the undelayed pulse output.
2. Delay LUT has the sync/filter on, and possibly a slower clock source than the system clock, and sync or filter is on, so this slows the signal down.
3. 2 or 4 clocks later, the output of the second LUT goes high. This resets the LUT as SR latch, and also gives you a delayed pulse you may need.
4. 2 or 4 clocks after that, reset goes low and and the mechanism is ready for the next pulse
5. You might be tempted to use an edge detector on the delay LUT for faster recovery time. This is not safe. If an incoming pulse beats your recovery time, you can end up with a wedged mechanism that can't be readily unwedged; consider this sequence of events:
  a. Input pulse sets SR latch, starting pulse output
  b. delay lut delays, and then generates a pulse
  c. Another input pulse arrives during the stretched pulse - starting either before or during that reset pulse, but ending after the reset pulse ends.
  e. There are three ways to handle the contradictory inputs. None of them work. If you set output low, the output will go low, but as soon as it goes away, set is still high, so the output is set once more. But the the brief low output could be missed by the synchronizer. This in turn would mean no reset pulse. If we set high on both, it's guaranteed not reset any time this happens. And maintaining the previous level just mirrors one of those. No matter what, there's a chance you won't get any more reset pulses whenever your recovery time is missed.
6. Without an edge detector, it depends on what behavior you want, specifically how you want to deal with a signal that occurs during your recovery time.

| On cont. inputs go | Throughout reset | During and after | Before and during |
|--------------------|------------------|------------------|-------------------|
| Low                | Pulse ends on <br/> time, one pulse length<br/>after end, 2nd pulse |  Pulse ends on <br/> time, one pulse length<br/>after end, 2nd pulse | Second pulse ignored. |
| High               | Pulse extra long <br/>, No second pulse | Pulse ends on <br/>time, second pulse <br/> starts immediately | First pulse lasts until<br/>second input ends |
| Maintain           | As high, above. | As low, above | As high, above. |



### Pulse Stretcher 2
This uses the sequencer. It doesn't get you the delayed pulse. However, it is largely free of complications.

#### Even LUT
Even LUT drives set.

INSEL:
* X: Pulse input
* Y: Link (they say you can't have S and R high on the SR latch (and we don't get to pick behavior there like we did. ))
* Masked

LUT 0x02

#### Odd LUT
The odd lut drives reset, much like the one above did.

INSEL:
X: Feedback (from Sequencer)
Y: Masked
Z: Clock or Masked

LUT: 0x02

Clock: Anything that 2 or 4 ticks of will be a long enough pulse

Sync/Filter: Sync to stretch any pulse on the input to 2 CLK<sub>odd</sub>, filter for 4 CLK<sub>odd</sub>

Sequencer: S/R latch


## More patterns

### A/B select
Allows selection of one of two inputs as it's output based on the remaining input. Obviously, chainable if you have to.

INSEL:
* X: Selector
* Y: A - output when selector low
* Z: B - output when selector high

LUT:
* 000: 0
* 001: 0
* 010: 1
* 011: 0
* 100: 0
* 101: 1
* 110: 1
* 111: 1

Ergo: TRUTH = 0xE4

### Gated Buffer
Let a signal through or output a constant signal depending on the second signal.
INSEL:
* X: D - When G is high, D is output
* Y: G - When G is low, the output is low.
* Z: masked

LUT:
* 000: 0
* 001: 0
* 010: 0
* 011: 1
Ergo: TRUTH = 0x08

Clock: N/A

Sync/Filter: Off


### Buffer (0 clock delay), 2 clock delay or 4 clock delay for level events
Without the synchronizer or prescaler, this is just a buffer (which is far from useless - you can use a LUT with this truth table and no synchronizer to get a level event onto a pin; For example, if you're on a 14-pin DD series. You've got the 3 PWM outputs coming from PC1-3, and the two TCD's on PD6 and PD7, you can't get any more PWM channels directly. If you're on a 14-pin tiny, and you've taken over TCA0 to do the 16-bit PWM that you need (TCA0 event output only works in SINGLE mode, not SPLIT mode, supposedly. I should probably check that the WO outputs aren't actually still being fed - that would make this trick go from niche to "widely useful", because any time that the pin on the low half of a TCA is used or missing, you could reclaim the PWM channels this way.)

INSEL:
* X: Input
* Y: Masked
* Z: Masked or used for clock source. Input signals are delayed 0, 2 or 4 of these clocks.

LUT:
* 000: 0
* 001: 1

TRUTH = 0x02

Clock: Whichever clock source will give you the delay that you need. This is CLK<sub>odd</sub>.

Sync/Filter: Off - 0 clock delay (buffer), Sync (2 clock delay) or Filter (4 clock delay, plus filtering).

Note that with sync/filter off, the delay is not zero, obviously, but the response time is significantly faster than the system clock. Not only that, but these two methods of setting up the buffer will give slightly different times:

1. Pin input -> Event channel -> CCL -> Pin output
2. Pin input -> CCL -> Pin output

This was investigated a while back using the much more convenient sandbox of a LUT oscillator (set to invert it's own output with no sync/filter)
3. CCL -> CCL -> CCL -> Pin output
4. CCL -> Event channel -> CCL -> Pin output
5. CCL output -> CCL -> Pin output
6. CCL -> Event channel -> CCL -> CCL -> Pin output.

In the second case, the idea was to exclude everything except the propagation time of the signal in the chip. It will oscillate, allowing a scope to be used to measure the frequency (hence the meaningful measure, the propagation delay). Note that bandwidth limiting needs to be off - scopes often are designed to attenuate signals above 20 MHz, and this is typically turned on by default. You usually want it on because it reduces the visibility of any high frequency noise that might be obscuring the signal, but not if you're looking at a signal above the ceiling (which this will be)).

When I tested it, 3 (lut pair, odd one takes the feedback as input, even one takes LINK as input, one echoes input unchanges (TRUTH=2) and the other inverts it (TRUTH=1). and 4 (odd lut, feeding an event channel used as it's input) generated the same frequency iirc, but 5 (even lut inverting own output) resulted in output twice as fast. Variant 6 (non-paired, consecutive luts, ie, first stage on LUT2, second on LUT1. LUT1 can take output from LUT2 via link, but to get it's output back to LUT2 needs an event channel) had a third the frequency of variant 5. The frequency of variant 5 was in the area of 80-100 MHz (!!!), indicating a propagation time per stage of 10-12ns (depending on temperature - dF<sub>LUT OSC</sub>/dT is orders of magnitude larger than dF<sub>Int. 20 MHz OSC</sub>/dT). I do not know what the speed of cases 1 and 2 are - but I predict that t<sub>1</sub> = A t<sub>5</sub>, t<sub>2</sub> = (A + B)t<sub>5</sub>, and that the values of A, B are either (2, 1), (2,2) or (3,2), and I think it's 2, 1.

### 2 LUT edge detector for RISING *or* FALLING
This generates a pulse two or four clocks long on either a rising *or* falling edge.

We need 2 LUTs, n, and n+1, where n is an even number.

#### LUTn+1: Configure for 2 or 4 clock delay as above
See also variant below. The first stage is just generating a delayed version of the input waveform.

#### LUTn
Then we take the input, and xor it with the delayed input, and output that.
INSEL:
* X: Input
* Y: LINK
* Z: Masked

LUT:
* 000: 0
* 001: 1
* 010: 1
* 011: 0
TRUTH = 0x06

Clock: N/A (typically)

Sync/Filter: (typically - can be used to delay the pulse slightly)

#### Variant: Lightning fast pulses
Using an input that is available through both the event system and directly (say, LUT0's PA1 input, which LUT0 can use directly, but LUT1 would need to get it through the event channel, and turn off sync/filter. The event channel (I believe) adds 1 propagation delay to the input on LUT1, which immediately switches it's output and that arrives at LUT0 through link 1 propagation time later) This adds a delay of 1 propagation time, and the CCL itself adds another. In this configuration like that you would get pulses of 20-25 ns duration whenever the input transitioned. Is this of much use? Doubtful - but it's useful to consider this limiting case.

#### Review - what determines what
There are two parameters needed to describe the behavior of this configuration - the delay between the rising edge of the input and the rising edge of the output is 1 propagation time (negligible almost all of the time), unless the filter or synchronizer of the EVEN lut is enabled, in which case that delay will be 2 or 4 CLK<sub>even</sub>. The length of the pulse is determined by CLK<sub>odd</sub> if the sync or filter are enabled - since the output is high when the inputs are different, a delay in the first stage manifests as the length of the pulse, and thus is 2 or 4 CLK<sub>odd</sub>. If the synchronizer or filter on the EVEN lut are used to delay the start of the pulse, F_CLK<sub>even</sub> must be >= F_CLK<sub>odd</sub> to avoid misbehavior, however, it may be as fast as you like within the limits of the hardware.

**Tip:** The AVR EB-series is coming with a very powerful PLL (for an AVR! But it seriously does: 8x or 16x multiply, 1, 2, 4 or 6 input divider, then an optional 2:1 divider on the output, and somewhere there's a prescaler B that divides something by 4 if you want. I don't know what and won't until we get the datasheet). And guess what - You can clock the CCL with it! So that's something to look forward to on the EB! For some of us, possibly the only thing, but let's not dwell on the negative.

## Prescaling clocks with the CCL

You can use CCL logic blocks to prescale a clock, albeit inefficiently. On parts with 6 LUTs, you can prescale by 2^18 if you're willing to use all LUTs and 3 event channels. This can be used for example to clock a TCB from a prescaled value that is slower than half the system clock but not used by any TCA.

Generally, I would argue that if you're using more than 3 LUTs, you should consider whether there is any other way to achieve your goal.

**Tip:** This will be way more powerful on the EB, for the reasons described in the tip at the end of the previous section.

### The general scheme

1. Find the prescaling factor you want on the table below.
2. Check the LUTS and EVENTS columns and make sure you have that many available.
  a. An additional event channel will be needed if you want to "skip" the more useful even lut in a pair\
  b. If ALL luts are used, and the chip is not impacted by errata about the link input on LUT3, AND you don't need the output on an event channel, you can save a channel. You can also save a channel if only using 2 LUTs.
3. Select a block of the necessary number of consecutive LUTs,
  a. The lowest LUT should be even.
4. Configure the LUTs:
  a. The highest number LUT should be set to use the output of the lowest number LUT, either through feedback (if only 2 luts used), link (if all luts are used for this) or otherwise an event channel set to use the lowest number LUT as generator.
  b. If the number under the column with the LUT number heading (0-5) is a 4, the "filter/synchronizer" should be set as synchronizer.
  c. If the number is an 8, it should be set as a filter.
  d. The highest number LUT should use the system clock (or other clock to be prescaled.
  e. If the operation is ADD set input 0 that lut to link, and the truth table to 0x02. (output a 1 when input is 1)
  f. If the operation in MUL set input 0 to be feedback (if even) or an event channel carrying it's own output (if odd), and input 2 to link (unless it's the highest lut).
  g. If the operation is ADDn, ADDnMULn+1, or MULn - Uh, I forget what exactly these translated into.
  h. The highest LUT, and any luts multiplying the previous one, should get a truth table of 0x01 (output a 1 when input is 0).
  i. Take the table with a spoonful of salt.  I found several errors in a short time looking at it.

### The giant table

| Total | p0 | 0 | 1 | oppr 0    | oppr 1    | p2+3 | p2 | 2 | 3 | oppr 2    | oppr 3    | p3 | 4 | 5 | op4 | LUTS | Events | Warning |
|-------|----|---|---|-----------|-----------|------|----|---|---|-----------|-----------|----|---|---|-----|------|--------|---------|
|     4 |  4 | 4 | - |     FIRST |         - |    - |  - | - | - |         - |         - |  - | - | - |   - |    2 |      0 |         |
|     8 |  8 | 4 | - |     FIRST |         - |    - |  - | - | - |         - |         - |  - | - | - |   - |    2 |      0 |         |
|    12 | 12 | 4 | 8 |       ADD |     FIRST |    - |  - | - | - |         - |         - |  - | - | - |   - |    2 |      0 |         |
|    16 | 16 | 8 | 8 |       ADD |     FIRST |    - |  - | - | - |         - |         - |  - | - | - |   - |    2 |      0 |         |
|    20 | 16 | 8 | 8 |       ADD |       ADD |    - |  4 | 4 | - |     FIRST |         - |  - | - | - |   - |    3 |      1 |         |
|    24 | 16 | 4 | 4 |      ADD2 |       ADD |    - |  8 | 8 | - |     FIRST |         - |  - | - | - |   - |    3 |      1 |         |
|    28 | 16 | 8 | 8 |       ADD |       ADD |    - | 12 | 8 | 4 |       ADD |     FIRST |  - | - | - |   - |    4 |      0 |         |
|    32 | 32 | 4 | 8 |       MUL |     FIRST |    - |  - | - | - |         - |         - |  - | - | - |   - |    2 |      0 |         |
|    36 |  4 | - | 4 |        -  |       ADD |    - | 32 | 8 | 4 |       MUL |     FIRST |  - | - | - |   - |    3 |      1 | ODD LOW |
|    40 |  8 | - | 8 |        -  |       ADD |    - | 32 | 8 | 4 |       MUL |     FIRST |  - | - | - |   - |    3 |      1 | ODD LOW |
|    44 | 12 | 4 | 8 |       ADD |       ADD |    - | 32 | 8 | 4 |       MUL |     FIRST |  - | - | - |   - |    4 |      2 |         |
|    48 | 16 | 8 | 8 |       ADD |       ADD |    - | 32 | 8 | 4 |       MUL |     FIRST |  - | - | - |   - |    4 |      1 |         |
|    52 | 32 | 4 | 8 |       MUL |       ADD |    - | 16 | 4 | 4 |       MUL |      ADD4 |  4 | 4 | - | FST |    5 |      1 |         |
|    64 | 64 | 8 | 8 |       MUL |     FIRST |    - |  - | - | - |         - |         - |  - | - | - |   - |    2 |      0 |         |
|    68 |  4 | - | 4 |        -  |       ADD |    - | 64 | 8 | 8 |       MUL |     FIRST |  - | - | - |   - |    3 |      1 | ODD LOW |
|    72 |  4 | - | 8 |        -  |       ADD |    - | 64 | 8 | 8 |       MUL |     FIRST |  - | - | - |   - |    3 |      1 | ODD LOW |
|    76 |  4 | 4 | 8 |       ADD |       ADD |    - | 64 | 8 | 8 |       MUL |     FIRST |  - | - | - |   - |    4 |      1 |         |
|    80 |  4 | 8 | 8 |       ADD |       ADD |    - | 64 | 8 | 8 |       MUL |     FIRST |  - | - | - |   - |    4 |      1 |         |
|    84 | 64 | 8 | 8 |       MUL |       ADD |    - | 16 | 8 | 8 |       ADD |      ADD4 |  4 | 4 | - | FST |    5 |      1 |         |
|    88 |  8 | 4 | 4 |       ADD |  ADD2MUL3 |    - |  * | 4 | 4 |       ADD |  ADD4MUL5 |  * | 4 | 4 |   - |    6 |      1 |         |
|    96 |  8 | 4 | 4 |       ADD |       ADD |    - | 12 | 4 | 8 |       ADD |      MUL4 |  4 | 4 | - |   - |    5 |      1 |         |
|   100 | 64 | 8 | 8 |       MUL |       ADD |    - | 32 | 4 | 8 |       MUL |      ADD4 |  4 | 4 | - |   - |    5 |      1 |         |
|   108 | 12 | 4 | 8 |       ADD |  ADD2MUL3 |    - |  * | 4 | 8 |       ADD |  ADD4MUL5 |  * | 4 | 4 | ADD |    6 |      1 |         |
|   128 | 16 | 4 | 4 |      MUL2 |       MUL |    - |  8 | 8 | - |     FIRST |         - |  - | - | - |   - |    3 |      1 |         |
|   140 | 12 | 4 | 8 |       ADD |       ADD |  128 | 16 | 8 | 8 |       ADD |       MUL |  8 | 4 | 4 | ADD |    6 |      0 |         |
|   144 | 12 | 4 | 8 |       ADD |       MUL |    - | 12 | 4 | 8 |       ADD |     FIRST |  - | - | - |   - |    4 |      1 |         |
|   152 |  8 | 4 | 4 |       ADD |  ADD2MUL3 |    - |  * | 4 | 4 |       ADD |  ADD4MUL5 |  * | 4 | 8 |   - |    6 |      1 |         |
|   160 |  8 | 4 | 4 |       ADD |       MUL |    - | 12 | 4 | 8 |       ADD |      ADD4 |  4 | 8 | - |   - |    5 |      1 |         |
|   172 | 12 | 4 | 8 |       ADD |  ADD2MUL3 |    - |  - | 4 | 8 |       ADD |  ADD4MUL5 |  * | 4 | 8 |   - |    6 |      1 |         |
|   192 | 16 | 8 | 8 |       ADD |       MUL |    - | 12 | 4 | 8 |       ADD |     FIRST |  - | - | - |   - |    4 |      1 |         |
|   204 | 12 | 4 | 8 |       ADD |       ADD |  192 | 16 | 8 | 8 |       ADD |       MUL | 12 | 4 | 8 | ADD |    6 |      0 |         |
|   208 | 16 | 8 | 8 |       ADD |  ADD2MUL3 |    - |  * | 8 | 8 |       ADD |  ADD4MUL5 |  * | 4 | 4 |   - |    6 |      1 |         |
|   224 | 32 | 4 | 8 |       MUL |       ADD |  192 | 16 | 8 | 8 |       ADD |       MUL | 12 | 4 | 8 | ADD |    6 |      0 |         |
|   256 | 16 | 4 | 4 |       MUL |       MUL |    - | 16 | 4 | 4 |       MUL |     FIRST |  - | - | - |   - |    4 |      1 |         |
|   268 | 12 | 4 | 8 |       ADD |       ADD |  256 | 16 | 8 | 8 |       ADD |       MUL | 16 | 8 | 8 | ADD |    6 |      0 |         |
|   288 | 12 | 4 | 8 |       ADD |       MUL |    - | 16 | 8 | 8 |       ADD |      ADD4 |  8 | 8 | - |   - |    5 |      1 |         |
|   300 | 12 | 4 | 8 |       ADD |  ADD2MUL3 |    - |  * | 4 | 8 |       ADD |  ADD4MUL5 |  * | 8 | 8 |   - |    6 |      2 |         |
|   336 | 16 | 8 | 8 |       ADD |  ADD2MUL3 |    - |  * | 8 | 8 |       ADD |  ADD4MUL5 |  * | 4 | 8 |   - |    6 |      2 |         |
|   384 | 16 | 8 | 8 |       ADD |       MUL |    - | 16 | 4 | 4 |       MUL |      ADD4 |  8 | 8 | - |   - |    5 |      1 |         |
|   512 | 64 | 8 | 8 |       MUL |       ADD |    - | 16 | 8 | 8 |       ADD |      MUL4 |  4 | 4 | - |   - |    5 |      1 |         |
|   576 | 12 | 4 | 8 |       ADD |       MUL |   48 | 32 | 4 | 8 |       MUL |       ADD | 16 | 4 | 4 | MUL |    6 |      2 |         |
|   592 | 16 | 8 | 8 |       ADD |  ADD2MUL3 |    - |  * | 8 | 8 |       ADD |  ADD4MUL5 |  * | 8 | 8 |   - |    6 |      1 |         |
|   640 | 16 | 4 | 4 |       MUL |       MUL |    - | 32 | 4 | 8 |       MUL |      ADD4 |  8 | 8 | - |   - |    5 |      2 |         |
|   768 | 12 | 4 | 8 |       ADD |       MUL |   64 | 32 | 4 | 8 |       MUL |       ADD | 32 | 4 | 8 | MUL |    6 |      2 |         |
|  1024 | 32 | 4 | 8 |       MUL |       MUL |    - | 32 | 4 | 8 |       MUL |     FIRST |  - | - | - |   - |    4 |      2 |         |
|  1088 | 16 | 4 | 4 |      MUL2 |       MUL |    - | 68 | 8 | 8 |       MUL |  ADD4MUL5 |  * | 8 | 4 | MUL |    5 |      3 |         |
|  1152 | 12 | 4 | 8 |       ADD |       MUL |    - | 16 | 8 | 8 |       ADD |      MUL4 |  8 | 8 | - |   - |    5 |      1 |         |
|  1280 | 16 | 4 | 4 |       MUL |       MUL |   80 | 16 | 4 | 4 |       MUL |       ADD | 64 | 8 | 8 | MUL |    6 |      2 |         |
|  1536 | 16 | 8 | 8 |       ADD |       MUL |    - | 16 | 4 | 4 |       MUL |      MUL4 |  8 | 8 | - |   - |    5 |      2 |         |
|  2048 | 16 | 4 | 4 |       MUL |       MUL |    - | 32 | 4 | 8 |       MUL |      MUL4 |  8 | 8 | - |   - |    5 |      2 |         |
|  2304 | 32 | 4 | 8 |       MUL |       MUL |    - | 64 | 8 | 8 |       MUL |      ADD4 |  8 | 8 | - |   - |    5 |      2 |         |
|  4096 | 64 | 8 | 8 |       MUL |       MUL |    - | 64 | 8 | 8 |       MUL |     FIRST |  - | - | - |   - |    4 |      2 |         |
|  4608 | 64 | 8 | 8 |       MUL |       MUL |    - | 64 | 8 | 8 |       MUL |      ADD4 | 12 | 8 | 4 | ADD |    5 |      2 |         |
|  8192 | 32 | 4 | 8 |       MUL |       MUL |    - | 64 | 8 | 8 |       MUL |      MUL4 |  8 | 8 | - |   - |    5 |      2 |         |
| 16384 | 16 | 8 | 8 |       ADD |       MUL | 1024 | 32 | 4 | 8 |       MUL |       MUL | 32 | 4 | 8 | MUL |    6 |      2 |         |
| 24576 | 12 | 4 | 8 |       ADD |       MUL | 2048 | 32 | 4 | 8 |       MUL |       MUL | 64 | 8 | 8 | MUL |    6 |      2 |         |
| 32768 | 64 | 8 | 8 |       MUL |       MUL |    - | 64 | 8 | 8 |       MUL |      MUL4 |  8 | 8 | - |   - |    5 |      2 |         |
| 65536 | 16 | 8 | 8 |       ADD |       MUL | 4096 | 64 | 8 | 8 |       MUL |       MUL | 64 | 8 | 8 | MUL |    6 |      3 |         |
|131072 | 64 | 8 | 8 |       MUL |       MUL | 2048 | 32 | 4 | 8 |       MUL |       MUL | 64 | 8 | 8 | MUL |    6 |      3 |         |
|262144 | 64 | 8 | 8 |       MUL |       MUL | 4096 | 64 | 8 | 8 |       MUL |       MUL | 64 | 8 | 8 | MUL |    6 |      3 |         |

`*` - This column is cannot express a value for the two combined pair of LUTs LUTs in the case where one of them is added to the lower LUTs, and the whole thing multiplied by the other.
