# Environment

## Windridge City

While we can't include assets from the asset store in our source code, you can easily import environments from the [Unity Asset Store](https://assetstore.unity.com/) such as [Windridge City](https://assetstore.unity.com/packages/3d/environments/roadways/windridge-city-132222). This portion of the guide explains how.

First, install the Post Processing plugin from the Unity Package Manager by opening `Window` -> `Package Manager`, finding the `Post Processing` plugin and click `Install`. Like so: 

```eval_rst
.. thumbnail:: /images/windridge-postprocessing.gif
```

Within Unity, open the asset store tab, download and import [Windridge City](https://assetstore.unity.com/packages/3d/environments/roadways/windridge-city-132222). After the scene has just been imported and opened, you screen should look like this:

```eval_rst
.. thumbnail:: /images/windridge-imported.gif
```

Once it's imported, in the `Project` tab, open the `Windridge City Demo Scene.unity`. Then, click on the `Main Camera` in the `Hierarchy` tab. Hold your cursor over the `Scene` view and press `F` on the keyboard to center the view.

Change the player rending `Color Space` to `Linear`, open `Edit` -> `Project Settings...` and on the left, select `Player` then under `Other Settings` change the `Colors Space*` setting to `Linear` and let it re-build.

```eval_rst
.. thumbnail:: /images/windridge-render-settings.gif
```

Now re-generate the lighting by opening `Windows` -> `Rendering` -> `Lighting Settings`. At the bottom of the `Lighting` tab, click 'Generate Lighting`. You'll see in the bottom-right corner of the scene a loading bar as lighting re-generates. This will take a few minutes (it took about 15 minutes on my machine) and all your CPU cores (hopefully not all your RAM though). We tested on a machine with 16GB of RAM and it worked fine. Your scene should look like this now:

```eval_rst
.. thumbnail:: /images/windridge-render.gif
```
