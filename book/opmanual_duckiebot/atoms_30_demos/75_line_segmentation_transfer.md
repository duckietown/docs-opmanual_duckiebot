# Line segmentation transfer {#line-segmentation-transfer status=beta}

This is a demo of line segmentation transfer, where a line segmentation model was trained on the simulator and trained to be able to segment real duckiebot video logs.

<div class='requirements' markdown="1">

Requires: Video log of Duckiebot

</div>

## Video of expected results {#line-segmentation-transfer-expected}

#### Using the transformed version
![line_sgmt_trsf_trsfmed](line_sgmt_trsf_trsfmed.gif)

#### Using the embedding version
![line_sgmt_trsf_embed](line_sgmt_trsf_embed.gif)

#### Using the live simulator demo
![line_sgmt_live_demo](line_sgmt_live_demo.gif)


## Duckietown setup notes {#line-segmentation-transfer-setup}

For the required setup, please refer to the full repo of this project. Link is [here](https://github.com/placaille/segmentation-transfer).

Briefly,

1) Download source code from [here](https://github.com/placaille/segmentation-transfer)
2) Create `conda` environment as instructed in [this section](https://github.com/placaille/segmentation-transfer#environment)

## Demo instructions {#line-segmentation-transfer-run}

### Line segmentation from video file

We provide two ways of segmenting a video file coming from the duckiebot log database.  Any video file from the duckietown logs database can be used for the demo. When launching any of them, our pre-trained models will be downloaded automatically and a `.gif` will be created after processing the images.

For more information as to how to get a video file from the duckietown database and to see which ones were used to train our models, see the [this section](https://github.com/placaille/segmentation-transfer#real-duckiebot-images).

 **Please ensure you have the necessary dependencies installed before launching the demo, see [this section](https://github.com/placaille/segmentation-transfer#environment) for more information**.

*WARNING: The processing can take some time, especially if not using a GPU and the video file is large. By default, we keep a high framerate to have smooth video while limiting the length of the input video to about 3 minutes. These can be changed in the source code, if necessary.*

#### Video transformed into simulator pixel space

This is a demo where the image is brought back to the full simulator pixel space before applying our line segmentation model. While this method is useful to see the transformed version of the real image into the simulator pixel space, performance is noticeably worst.

To process a video use the `make gif-transformed` recipe. For example,

```
make gif-transformed input_file=duckie_video.mp4 output_file=duckie_transformed.gif
```

#### Video compressed into embedding space

This is a demo where the image is compressed into features from where the line segmentation model is applied. While this method offers less interpretability, performance is noticeably better.

To process a video use the `make gif-embedding` recipe. For example,
```
make gif-embedding input_file=duckie_video.mp4 output_file=duckie_embedding.gif
```

### Running the segmentation demo on the duckietown simulator

We provide a tool to test our pre-trained segmentation network live on the simulator while the user can manually control the agent. In order to run the simulation, simply run `$ python src/results/seg_control.py`.

**Please ensure you have the necessary dependencies installed before launching the demo, see [this section](https://github.com/placaille/segmentation-transfer#environment) for more information**.

*WARNING: We tested the live demo on OSX. It might possibly not work on Linux out of the box.*
