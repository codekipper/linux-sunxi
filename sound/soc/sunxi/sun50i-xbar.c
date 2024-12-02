#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/reset.h>

// Regmap configuration for xbar registers
static const struct regmap_config xbar_regmap_config = {
    .reg_bits = 32,                // 32-bit register width
    .val_bits = 32,                // 32-bit value width
    .max_register = 0xc,           // Max register offset (based on your regmap size)
    .reg_stride = 4,               // 4-byte register stride (for 32-bit registers)
    .cache_type = REGCACHE_NONE,
};

static int xbar_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct platform_device *parent_pdev = to_platform_device(dev->parent);
    struct resource *res;
    void __iomem *base;
    struct regmap *regmap;
    int ret;

    dev_info(dev, "XBAR device started probing\n");
#if 1
    // Step 2: Map the child device's resource using the parent device's memory region
    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!res) {
        dev_err(dev, "Failed to get parent resource\n");
        return -ENODEV;
    }

    // Step 2: Map the child device's resource using the parent device's memory region
    base = devm_ioremap(&pdev->dev, res->start, 0xc);  // Offset and size specific to child device
#else
    base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
#endif
    if (IS_ERR(base)) {
        dev_err(dev, "Failed to map memory for APBIF\n");
        return PTR_ERR(base);
    }
    dev_info(dev, "XBAR device probed, mapped base: %p\n", base);
    // Step 3: Initialize regmap for the child device
    regmap = devm_regmap_init_mmio(dev, base, &xbar_regmap_config);
    if (IS_ERR(regmap)) {
        dev_err(dev, "Failed to initialize regmap for ABAR\n");
        return PTR_ERR(regmap);
    }

    // Step 4: Use regmap to access registers (Example: write and read registers)
    ret = regmap_write(regmap, 0xc, 0x80400000);  // Write to a register
    if (ret) {
        dev_err(dev, "Failed to write to register\n");
        return ret;
    }

    u32 value;
    ret = regmap_read(regmap, 0xc, &value);  // Read from a register
    if (ret) {
        dev_err(dev, "Failed to read from register\n");
        return ret;
    }

    // Print the resource address and size for debugging
    dev_info(dev, "XBAR resource: start=0x%llx, size=0x%llx\n",
             (unsigned long long)res->start, (unsigned long long)resource_size(res));
    dev_info(dev, "XBAR device probed successfully\n");
    return 0;
}

static void xbar_remove(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;

    dev_info(dev, "XBAR device removed successfully\n");
}

static const struct of_device_id xbar_of_match[] = {
    { .compatible = "allwinner,sun50i-h616-xbar", },
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, xbar_of_match);

static struct platform_driver xbar_driver = {
    .probe = xbar_probe,
    .remove = xbar_remove,
    .driver = {
        .name = "xbar",
        .of_match_table = xbar_of_match,
    },
};

module_platform_driver(xbar_driver);

MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("AHUB driver");
MODULE_LICENSE("GPL");
