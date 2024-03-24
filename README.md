# Everything Controller V2.0

This project is an updated version of my Everything Controller I did a while back. This time I focused on transmission time, using the powerfull Teensy 3.2 as the main microcontroller. I also added more inputs, so I have even more functions to help the control of various projects, and not only robots.

![Finished project](https://github.com/Nabinho/Everything-Controller-V2.0/blob/main/Photos/front_min.jpg =349x465 "Finished project")

The PCB and base code of this project are available for replication. The Teensy 3.2 is discontinued, so if you don't have one, maybe this can be a great reference for your own project.

## BOM

The SMD components list can be found in the interactive BOM generated with the KiCad project. Besides the Teensy 3.2 I had for a while, the rest of the THT components I bought from Aliexpress and all of the links are listed below.

* 1x - [EByte E01](https://pt.aliexpress.com/item/1005001803228202.html?spm=a2g0o.order_list.order_list_main.110.3876caa4MgzPN5&gatewayAdapt=glo2bra);
* 1x - [Waveshare SSD1327 128x128 OLED Display](https://pt.aliexpress.com/item/32853228170.html?spm=a2g0o.order_list.order_list_main.22.3876caa4MgzPN5&gatewayAdapt=glo2bra);
* 2x - [10k Ohm Rotary Potentiometer](https://pt.aliexpress.com/item/1005005956593595.html?spm=a2g0o.order_list.order_list_main.34.3876caa4MgzPN5&gatewayAdapt=glo2bra);
* 2x - [10k Ohm Slider Potentiometer](https://pt.aliexpress.com/item/1005006075347058.html?spm=a2g0o.order_list.order_list_main.35.3876caa4MgzPN5&gatewayAdapt=glo2bra);
* 2x - [EC11 Rotary Encoder](https://pt.aliexpress.com/item/1005005882895811.html?spm=a2g0o.order_list.order_list_main.36.3876caa4MgzPN5&gatewayAdapt=glo2bra);
* 4x - [Cherry MX Switches (any color)](https://pt.aliexpress.com/item/1005006255961111.html?spm=a2g0o.cart.0.0.360c7f06wTXtF8&mp=1&gatewayAdapt=glo2bra);
* 2x - [PS5 Hall Effect Joystick](https://pt.aliexpress.com/item/1005006165745072.html?spm=a2g0o.order_list.order_list_main.58.3876caa4MgzPN5&gatewayAdapt=glo2bra);

Besides the electronics components, a pair of 18650 Li-Ion batteries is necessary to power the controller.

Some screws and nuts ares also recommended to mount the soldered PCB in an acrylic or 3D printed backplate.

## Bechmark Tests

As soon as the controller was finished I ran some benchmark tests to validate the controller performance. Bellow are the tests I ran so far and the results.

| Tests | Results |
| ------------- |:-------------:|
| Battery feedback reading | The battery feedback circuit from the controller had a precision of 0.3 mV most of the time, but I had to include a "debounce" for the OLED update, due to some voltage fluctuation |
| Transmission latency | With a simple receiver just connected to the NRF24L01 transceiver and only calculating the time between messages (data sent and acknolegement) the average time was 3 ms during an 1 hour test |
| Battery duration | Using a pair of 3000 mAh 18650 Li-Ion batteries the controller stayed turned on during 21 hours (average consumption of 150 mA) |
| Transmission range | This test is still TBD, however I want to see if the optimistic expectations from EByte of a transmission range of 5 km is true |

### Conclusion/Contact

As this is a personal project, I would love to see if anyone replicate or takes inspiration from this project. So if you replicate or use this as a base for your own project, please share it with me in Instagram.
