#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/reset.h>

static struct regmap *ahub_regmap;  // Define regmap for ahub

// Regmap configuration for AHUB registers
static const struct regmap_config ahub_regmap_config = {
    .reg_bits = 32,                // 32-bit register width
    .val_bits = 32,                // 32-bit value width
    .max_register = 0xc,           // Max register offset (based on your regmap size)
    .reg_stride = 4,               // 4-byte register stride (for 32-bit registers)
    .cache_type = REGCACHE_NONE,
};

static int ahub_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct resource *res;
    struct clk *audio_hub_clk;
    struct clk *apb_clk;
    void __iomem *base;
    struct reset_control *rst;
    int ret;

    // Step 1: Get the resource from the platform device (the memory region)
    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!res) {
        dev_err(dev, "Failed to get memory resource\n");
        return -ENODEV;
    }

    // Print the resource address and size for debugging
    dev_info(dev, "AHUB resource: start=0x%llx, size=0x%llx\n",
             (unsigned long long)res->start, (unsigned long long)resource_size(res));

    // Step 2: Map the memory region for use
    base = devm_ioremap_resource(dev, res);
    if (IS_ERR(base)) {
        dev_err(dev, "Failed to map memory\n");
        return PTR_ERR(base);
    }

    // Initialize regmap for AHUB using the mapped base address
    ahub_regmap = devm_regmap_init_mmio(dev, base, &ahub_regmap_config);
    if (IS_ERR(ahub_regmap)) {
        dev_err(dev, "Failed to initialize regmap\n");
        return PTR_ERR(ahub_regmap);
    }

    // Step 3: Get the clock for the AHUB device
    apb_clk = devm_clk_get(dev, "apb");
    if (IS_ERR(apb_clk)) {
        dev_err(dev, "Failed to get clock\n");
        return PTR_ERR(apb_clk);
    }

    // Enable the clock
    ret = clk_prepare_enable(apb_clk);
    if (ret) {
        dev_err(dev, "Failed to enable clock\n");
        return ret;
    }
    ret = clk_prepare_enable(audio_hub_clk);
    if (ret) {
        dev_err(dev, "Failed to enable clock\n");
        return ret;
    }
    audio_hub_clk = devm_clk_get(dev, "audio-hub");
    if (IS_ERR(audio_hub_clk)) {
        dev_err(dev, "Failed to get clock\n");
        return PTR_ERR(audio_hub_clk);
    }

    // Enable the clock
    ret = clk_prepare_enable(audio_hub_clk);
    if (ret) {
        dev_err(dev, "Failed to enable clock\n");
        return ret;
    }

	rst = devm_reset_control_get_optional_exclusive(&pdev->dev,
							      NULL);
	if (PTR_ERR(rst) == -EPROBE_DEFER) {
		ret = -EPROBE_DEFER;
		dev_err(&pdev->dev, "Failed to get reset: %d\n", ret);
		return ret;
	}
	if (!IS_ERR(rst)) {
		reset_control_deassert(rst);
 		dev_info(dev, "AHUB device reset deasserted\n");
         }
#if 0
    // Allocate the child device (apbif)
    struct platform_device *child_pdev = platform_device_alloc("apbif", -1);
    if (!child_pdev) {
        dev_err(dev, "Failed to allocate apbif device\n");
        return -ENOMEM;
    }

    // Add the child device (apbif)
    ret = platform_device_add(child_pdev);
    if (ret) {
        dev_err(dev, "Failed to add apbif device\n");
        platform_device_put(child_pdev);
        return ret;
    }
#endif
    dev_info(dev, "AHUB device probed successfully\n");
    return 0;
}

static void ahub_remove(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct clk *audio_hub_clk;

    // Step 1: Get the clock for the AHUB device
    audio_hub_clk = devm_clk_get(dev, "audio-hub");
    if (!IS_ERR(audio_hub_clk)) {
        // Step 2: Disable the clock if it was enabled
        clk_disable_unprepare(audio_hub_clk);
    }

    dev_info(dev, "AHUB device removed successfully\n");
}

static const struct of_device_id ahub_of_match[] = {
    { .compatible = "allwinner,sun50i-h616-audio-hub", },
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ahub_of_match);

static struct platform_driver ahub_driver = {
    .probe = ahub_probe,
    .remove = ahub_remove,
    .driver = {
        .name = "ahub",
        .of_match_table = ahub_of_match,
    },
};

module_platform_driver(ahub_driver);

MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("AHUB driver");
MODULE_LICENSE("GPL");
