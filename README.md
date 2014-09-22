nrf-beacon
==========

Please do

```bash
git submodule init
git submodule update
```

after cloning to fetch the xpcc library.


Then, please manually edit __driver.xml__ in

    xpcc/src/xpcc/architecture/platform/peripheral/uart/stm32/

set **tx_buffer to 254**