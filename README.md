# adxl-355z-drv
Driver for accelerometer ADXL355Z

# How to use
## Step 1
Implemetation SPI read write reg

```c
void bsp_adxl355_spi_read(const uint8_t reg, uint8_t *const value, const uint16_t size)
{
  // You implementation
}

void bsp_adxl355_spi_write(const uint8_t reg, uint8_t *const value, const uint16_t size)
{
  // You implementation
}
```

## Step 2
Create ADXL355Z handle
```c
const adxl355_t adxl_355z_handle =
{
 .read = bsp_adxl355_spi_read,
 .write = bsp_adxl355_spi_write
};
```

## Step 2
Use driver functions

```c
// Read dev_mst_id, dev_part_id, dev_rev_id
static  adxl355__general_info_t gen_info = {0};
adxl355__get_general_info(&adxl_355z_handle, &gen_info);
```
