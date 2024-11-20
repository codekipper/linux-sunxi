#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/regmap.h>
#include <linux/of.h>

static int apbif_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    //struct platform_device *parent_pdev = to_platform_device(dev->parent);
    struct resource *res;
    void __iomem *base;
    struct regmap *regmap;
    struct regmap_config regmap_config = {
        .reg_bits = 32,
        .val_bits = 32,
        .max_register = 0x174,  // Example maximum register offset
    };
    int ret;

    dev_info(dev, "APBIF device started probing\n");
#if 0
    // Step 1: Get the parent's allocated resource (memory region)
    res = platform_get_resource(parent_pdev, IORESOURCE_MEM, 0);
    if (!res) {
        dev_err(dev, "Failed to get parent resource\n");
        return -ENODEV;
    }

    // Step 2: Map the child device's resource using the parent device's memory region
    base = devm_ioremap(dev, res->start + 0x10, 0x174);  // Offset and size specific to child device
    if (IS_ERR(base)) {
        dev_err(dev, "Failed to map memory for APBIF\n");
        return PTR_ERR(base);
    }

    dev_info(dev, "APBIF device probed, mapped base: %p\n", base);
    // Step 3: Initialize regmap for the child device
    regmap = devm_regmap_init_mmio(dev, base, &regmap_config);
    if (IS_ERR(regmap)) {
        dev_err(dev, "Failed to initialize regmap for APBIF\n");
        return PTR_ERR(regmap);
    }

    // Step 4: Use regmap to access registers (Example: write and read registers)
    ret = regmap_write(regmap, 0x10, 0x12345678);  // Write to a register
    if (ret) {
        dev_err(dev, "Failed to write to register\n");
        return ret;
    }

    u32 value;
    ret = regmap_read(regmap, 0x10, &value);  // Read from a register
    if (ret) {
        dev_err(dev, "Failed to read from register\n");
        return ret;
    }

    dev_info(dev, "Read 0x%08x from register 0x10\n", value);

#endif
    dev_info(dev, "APBIF device probed successfully\n");
    return 0;
}

static void apbif_remove(struct platform_device *pdev)
{
    dev_info(&pdev->dev, "APBIF device removed successfully\n");
}

static const struct of_device_id apbif_of_match[] = {
    { .compatible = "allwinner,sun50i-h616-apbif", },
    { .compatible = "allwinner,sun50i-h6-apbif", },
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, apbif_of_match);

static struct platform_driver apbif_driver = {
    .probe = apbif_probe,
    .remove = apbif_remove,
    .driver = {
        .name = "apbif",
        .of_match_table = apbif_of_match,
    },
};

module_platform_driver(apbif_driver);

MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("APBIF driver");
MODULE_LICENSE("GPL");
