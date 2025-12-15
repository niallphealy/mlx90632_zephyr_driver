# MLX90632 Zephyr Driver

Driver for the <a href="https://www.melexis.com/en/documents/documentation/datasheets/datasheet-mlx90632">Melexis MLX90632</a> infrared temperature sensor for <a href="https://docs.zephyrproject.org/latest/index.html">Zephyr RTOS</a>.

Desgined to work with the <a href="https://www.nordicsemi.com/Products/Development-software/nRF-Connect-SDK">nRF Connect SDK</a>.

To include in a project, add the following lines to your west manifest

    - name: mlx90632
      path: modules/mlx90632
      revision: main
      url: https://github.com/IMMEDesign/mlx90632_zephyr_driver-main.git

The run `west update` in an nRF Connect terminal.
