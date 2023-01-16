The audio hub code here is based on the Nvidia Jetson code that has a similar setup for routing audio to various audio blocks.
https://docs.nvidia.com/jetson/archives/r34.1/DeveloperGuide/text/SD/Communications/AudioSetupAndDevelopment.html#audio-hub-hardware-architecture

Device tree settings
https://elixir.bootlin.com/linux/v6.12-rc2/source/arch/arm64/boot/dts/nvidia/tegra234.dtsi#L164
aconnect - bus details that just gets the clocks and populated the platform_devices from device tree data (I've skipped this part and moved the functionality into th ahub code.)
https://elixir.bootlin.com/linux/v6.12-rc2/source/drivers/bus/tegra-aconnect.c

audio-graph-card - Get's the DAI's and clocks.
This is based on generic audio graph card driver along with additional
customizations for Tegra platforms. It uses the same bindings with
additional standard clock DT bindings required for Tegra.

https://elixir.bootlin.com/linux/v6.12-rc2/source/arch/arm64/boot/dts/nvidia/tegra210-p3450-0000.dts#L1894
https://elixir.bootlin.com/linux/v6.12-rc2/source/Documentation/devicetree/bindings/sound/nvidia,tegra-audio-graph-card.yaml
https://elixir.bootlin.com/linux/v6.12-rc2/source/sound/soc/tegra/tegra_audio_graph_card.c

I will replicate its functionality using
https://github.com/codekipper/linux-sunxi/blob/sunxi-wip/arch/arm64/boot/dts/allwinner/sun50i-h313-x96q-pro.dts#L92
and
https://github.com/codekipper/linux-sunxi/blob/sunxi-wip/sound/soc/sunxi/sun50i-audio-graph-card.c

ahub - 
The Audio Hub (AHUB) comprises a collection of hardware accelerators
for audio pre-processing, post-processing and a programmable full
crossbar for routing audio data across these accelerators. It has
external interfaces such as I2S, DMIC, DSPK. It interfaces with ADMA
engine through ADMAIF.
  
https://elixir.bootlin.com/linux/v6.12-rc2/source/arch/arm64/boot/dts/nvidia/tegra210.dtsi#L1381
https://elixir.bootlin.com/linux/v6.12-rc2/source/Documentation/devicetree/bindings/sound/nvidia,tegra210-ahub.yaml
https://elixir.bootlin.com/linux/v6.12-rc2/source/sound/soc/tegra/tegra210_ahub.c#L1315

ports in the dtsi show the relationship between the devices.
https://elixir.bootlin.com/linux/v6.12-rc2/source/arch/arm64/boot/dts/nvidia/tegra210.dtsi#L1723

I will replicate its functionality using
https://github.com/codekipper/linux-sunxi/blob/sunxi-wip/arch/arm64/boot/dts/allwinner/sun50i-h616.dtsi#L1055
and
https://github.com/codekipper/linux-sunxi/blob/sunxi-wip/sound/soc/sunxi/sun50i-audio-hub.c

Registers to manipulate
AHUB Control Register      AHUB_CTRL 0x0 bit 4 HDMI_SRC_SELECT
AHUB Reset Register        AHUB_RST 0x8 De-assert resets
AHUB Clock Gating Register AHUB_GAT 0xC Turn clocks on


admaif
ADMAIF is the interface between ADMA and AHUB. Each ADMA channel
that sends/receives data to/from AHUB must interface through an
ADMAIF channel. ADMA channel sending data to AHUB pairs with ADMAIF
Tx channel and ADMA channel receiving data from AHUB pairs with
ADMAIF Rx channel.

https://elixir.bootlin.com/linux/v6.12-rc2/source/arch/arm64/boot/dts/nvidia/tegra210.dtsi#L1394
https://elixir.bootlin.com/linux/v6.12-rc2/source/Documentation/devicetree/bindings/sound/nvidia,tegra210-admaif.yaml
https://elixir.bootlin.com/linux/v6.12-rc2/source/sound/soc/tegra/tegra210_admaif.c

I will replicate its functionality using
https://github.com/codekipper/linux-sunxi/blob/sunxi-wip/arch/arm64/boot/dts/allwinner/sun50i-h616.dtsi#L983
and
https://github.com/codekipper/linux-sunxi/blob/sunxi-wip/sound/soc/sunxi/sun50i-apbif.c
Registers to manipulate
https://github.com/codekipper/linux-sunxi/blob/sunxi-wip/sound/soc/sunxi/sun50i-apbif.c#L20C41-L20C43

i2s
The Inter-IC Sound (I2S) controller implements full-duplex,
bi-directional and single direction point-to-point serial
interfaces. It can interface with I2S compatible devices.
I2S controller can operate both in master and slave mode.

https://elixir.bootlin.com/linux/v6.12-rc2/source/arch/arm64/boot/dts/nvidia/tegra210.dtsi#L1505
https://elixir.bootlin.com/linux/v6.12-rc2/source/arch/arm64/boot/dts/nvidia/tegra234.dtsi#L204 (Better example with ports declared)
https://elixir.bootlin.com/linux/v6.12-rc2/source/Documentation/devicetree/bindings/sound/nvidia,tegra210-i2s.yaml
https://elixir.bootlin.com/linux/v6.12-rc2/source/sound/soc/tegra/tegra210_i2s.c

I will replicate its functionality using
https://github.com/codekipper/linux-sunxi/blob/sunxi-wip/arch/arm64/boot/dts/allwinner/sun50i-h616.dtsi#L1034
https://github.com/codekipper/linux-sunxi/blob/sunxi-wip/arch/arm64/boot/dts/allwinner/sun50i-h313-x96q-pro.dts#L113C2-L113C11
and
https://github.com/codekipper/linux-sunxi/blob/sunxi-wip/sound/soc/sunxi/sun50i-i2s.c
(similar to the sun4i-i2s.c driver)


+---------------------------------------------------+
|        Allwinner AHUB memory map                  |
+---------------------------------------------------+
|  Address     |  Size  |  Description              |
+---------------------------------------------------+
|  0x5097000   |  0x10  |  AHUB (sun50i-audio-hub)  |
|  0x5097010   |  0x30  |  APBIF0 TX (sun50i-apbif) |
|  0x5097040   |  0x30  |  APBIF1 TX (sun50i-apbif) |
|  0x5097070   |  0x30  |  APBIF2 TX (sun50i-apbif) |
|  0x5097100   |  0x30  |  APBIF0 RX (sun50i-apbif) |
|  0x5097130   |  0x30  |  APBIF1 RX (sun50i-apbif) |
|  0x5097160   |  0x30  |  APBIF2 RX (sun50i-apbif) |
|  0x5097200   |  0x100 |  I2S0 (sun50i-i2s)        |
|  0x5097300   |  0x100 |  I2S1 (sun50i-i2s)        |
|  0x5097400   |  0x100 |  I2S2 (sun50i-i2s)        |
|  0x5097500   |  0x100 |  I2S3 (sun50i-i2s)        |
|  0x5097A00   |  0x100 |  DAM0 (NA)                |
|  0x5097A80   |  0x80  |  DAM1 (NA)                |
|  ...         |  ...   |  ...                      |
|  0x5097FFF   |  ...   |  ...                      |
+---------------------------------------------------+
