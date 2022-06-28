/*
 * Copyright 2018-2019 Synergy Team, http://synergy.msk.ru
 * NXP i.MX6 NAU8822 codec ASoC machine driver
 */

#include <linux/clk.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <sound/soc.h>

#include "../codecs/nau8822.h"

#include "fsl_sai.h"

#define DAI_NAME_SIZE 32

struct imx_nau8822_data {
  struct snd_soc_dai_link dai;
  struct snd_soc_card card;
  char codec_dai_name[DAI_NAME_SIZE];
  char platform_name[DAI_NAME_SIZE];
  struct clk* codec_clk;
  unsigned int clk_frequency;
  bool is_codec_master;
};

static int imx_nau8822_dai_init(struct snd_soc_pcm_runtime* rtd) {
  struct imx_nau8822_data* data = snd_soc_card_get_drvdata(rtd->card);
  struct device* dev = rtd->card->dev;
  int ret;

  if (!data->is_codec_master) {
    ret = snd_soc_dai_set_sysclk(rtd->codec_dai, 0, data->clk_frequency,
                                 SND_SOC_CLOCK_OUT);
    if (ret) {
      dev_err(dev, "could not set codec driver clock params\n");
      return ret;
    }
  } else {
    ret = snd_soc_dai_set_sysclk(rtd->codec_dai, 0, data->clk_frequency,
                                 SND_SOC_CLOCK_IN);
    if (ret) {
      dev_err(dev, "could not set codec driver clock params\n");
      return ret;
    }
  }

  return 0;
}

static const struct snd_soc_dapm_widget imx_nau8822_dapm_widgets[] = {
    SND_SOC_DAPM_SPK("Speaker", NULL),
    SND_SOC_DAPM_MIC("Mic", NULL),
};

static int imx_nau8822_probe(struct platform_device* pdev) {
  struct device_node* np = pdev->dev.of_node;
  struct device_node *cpu_np, *codec_np;
  struct platform_device* cpu_pdev;
  struct i2c_client* codec_dev;
  struct imx_nau8822_data* data = NULL;
  int ret;

  cpu_np = of_parse_phandle(pdev->dev.of_node, "cpu-dai", 0);
  if (!cpu_np) {
    dev_err(&pdev->dev, "cpu dai missing or invalid\n");
    ret = -EINVAL;
    goto fail;
  }

  codec_np = of_parse_phandle(pdev->dev.of_node, "audio-codec", 0);
  if (!codec_np) {
    dev_err(&pdev->dev, "audio codec missing or invalid\n");
    ret = -EINVAL;
    goto fail;
  }

  cpu_pdev = of_find_device_by_node(cpu_np);
  if (!cpu_pdev) {
    dev_err(&pdev->dev, "failed to find SAI platform device\n");
    ret = -EINVAL;
    goto fail;
  }

  codec_dev = of_find_i2c_device_by_node(codec_np);
  if (!codec_dev || !codec_dev->dev.driver) {
    dev_err(&pdev->dev, "failed to find codec platform device\n");
    ret = -EINVAL;
    goto fail;
  }

  data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
  if (!data) {
    ret = -ENOMEM;
    goto fail;
  }

  if (of_property_read_bool(pdev->dev.of_node, "codec-master"))
    data->is_codec_master = true;

  data->codec_clk = devm_clk_get(&codec_dev->dev, "mclk");
  if (IS_ERR(data->codec_clk)) {
    ret = PTR_ERR(data->codec_clk);
    dev_err(&pdev->dev, "failed to get codec clk: %d\n", ret);
    goto fail;
  }

  data->clk_frequency = clk_get_rate(data->codec_clk);

  data->dai.name = "HiFi";
  data->dai.stream_name = "HiFi";
  data->dai.codec_dai_name = "nau8822-HiFi";
  data->dai.codec_of_node = codec_np;
  data->dai.cpu_of_node = cpu_np;
  data->dai.platform_of_node = cpu_np;
  data->dai.init = &imx_nau8822_dai_init;
  data->dai.dai_fmt =
      SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS;

  data->card.dev = &pdev->dev;
  ret = snd_soc_of_parse_card_name(&data->card, "model");
  if (ret)
    goto fail;

  data->card.num_links = 1;
  data->card.owner = THIS_MODULE;
  data->card.dai_link = &data->dai;
  data->card.dapm_widgets = imx_nau8822_dapm_widgets;
  data->card.num_dapm_widgets = ARRAY_SIZE(imx_nau8822_dapm_widgets);

  platform_set_drvdata(pdev, &data->card);
  snd_soc_card_set_drvdata(&data->card, data);

  ret = devm_snd_soc_register_card(&pdev->dev, &data->card);
  if (ret) {
    dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);
    goto fail;
  }

  of_node_put(cpu_np);
  of_node_put(codec_np);

fail:
  if (data && !IS_ERR(data->codec_clk))
    clk_put(data->codec_clk);
  if (cpu_np)
    of_node_put(cpu_np);
  if (codec_np)
    of_node_put(codec_np);

  return ret;
}

static int imx_nau8822_remove(struct platform_device* pdev) {
  struct snd_soc_card* card = platform_get_drvdata(pdev);
  struct imx_nau8822_data* data = snd_soc_card_get_drvdata(card);
  clk_put(data->codec_clk);

  if (data->codec_clk) {
    clk_disable_unprepare(data->codec_clk);
    clk_put(data->codec_clk);
  }
  snd_soc_unregister_card(&data->card);

  return 0;
}

static const struct of_device_id imx_nau8822_dt_ids[] = {
    {
        .compatible = "fsl,imx-audio-nau8822",
    },
    {
        .compatible = "fsl,imx-audio-nau8822",
    },
    {/* sentinel */}};
MODULE_DEVICE_TABLE(of, imx_nau8822_dt_ids);

static struct platform_driver imx_nau8822_driver = {
    .driver =
        {
            .name = "imx-nau8822",
            .owner = THIS_MODULE,
            .of_match_table = imx_nau8822_dt_ids,
        },
    .probe = imx_nau8822_probe,
    .remove = imx_nau8822_remove,
};
module_platform_driver(imx_nau8822_driver);

MODULE_AUTHOR("Synergy Team, http://synergy.msk.ru");
MODULE_DESCRIPTION("NXP i.MX NAU8822 codec ASoC machine driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:imx-nau8822");
