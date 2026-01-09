# Lekiwi Robot 3D Visualizer

Interactive 3D viewer for the Lekiwi omnidirectional robot, built with Three.js and urdf-loader. Automatically parses URDF and loads STL meshes from GitHub.

## Files

- **`codepen_github.html`** - CodePen-ready viewer using urdf-loader

## Quick Start

1. Go to [CodePen.io](https://codepen.io/pen/) and create a new pen

2. **Add required libraries** (Settings → JS → Add External Scripts):
   Add these URLs in order:
   - `https://cdn.jsdelivr.net/npm/three@0.128.0/build/three.min.js`
   - `https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js`
   - `https://cdn.jsdelivr.net/npm/urdf-loader@0.12.6/umd/URDFLoader.js`

3. Open `codepen_github.html` in a text editor

4. Copy the three sections to CodePen:
   - **HTML section** → HTML panel
   - **CSS section** (remove `<!--` and `-->`) → CSS panel
   - **JavaScript section** (remove `<!--` and `-->`) → JS panel

5. **No preprocessor needed!** The robot will load automatically

> **Note**: Uses urdf-loader to parse URDF and load meshes from `adityakamath/lekiwi_ros2`

## Features

- 🤖 **URDF-based** - Automatically loads robot from URDF definition
- 🎯 **Interactive controls**:
  - Left click + drag: Rotate camera
  - Right click + drag: Pan camera
  - Scroll wheel: Zoom in/out
- 📐 **Visual aids**: Toggle ROS-style coordinate axes and grid
- 📷 **Camera presets**: Reset, top view, and side view buttons
- 🎨 **Accurate rendering**: Light grey base, charcoal sensors/wheels
- 🔄 **ROS coordinate system**: X (red) forward, Y (green) left, Z (blue) up
- 📷 **Camera presets**: Reset, top view, and side view buttons
- 🎨 **Accurate colors**: Light grey base, charcoal wheels/sensors
- 💡 **Professional lighting**: Ambient + directional lights with shadows

## Robot Components

The visualizer displays:

- **Base**: Triangular platform (light grey)
- **Wheels**: 3 omni wheels at 120° intervals (charcoal)
  - Left wheel: 60° from +X axis
  - Back wheel: 180° from +X axis
  - Right wheel: 300° from +X axis
- **LiDAR**: LD06 sensor on top (charcoal)
- **Camera**: HD webcam at front (charcoal)

## Embedding in Your Website

### Basic Embed

```html
<iframe
  src="https://your-domain.com/lekiwi_viewer.html"
  width="800"
  height="600"
  frameborder="0"
  style="border: none; border-radius: 8px;">
</iframe>
```

### CodePen Embed

Once you've created your CodePen:

1. Click the "Share" button in CodePen
2. Copy the embed code
3. Paste it into your website

Example:
```html
<p class="codepen" data-height="600" data-default-tab="result" data-slug-hash="YOUR_PEN_ID" data-user="YOUR_USERNAME">
  <span>See the Pen <a href="https://codepen.io/YOUR_USERNAME/pen/YOUR_PEN_ID">Lekiwi Robot</a></span>
</p>
<script async src="https://cpwebassets.codepen.io/assets/embed/ei.js"></script>
```

### Responsive Embed

```html
<div style="position: relative; padding-bottom: 75%; height: 0; overflow: hidden;">
  <iframe
    src="https://your-domain.com/lekiwi_viewer.html"
    style="position: absolute; top: 0; left: 0; width: 100%; height: 100%; border: none;">
  </iframe>
</div>
```

## Customization

### Change Colors

Edit the material definitions in the JavaScript:

```javascript
const lightGreyMaterial = new THREE.MeshStandardMaterial({
  color: 0xcccccc,  // Change this hex color
  metalness: 0.3,
  roughness: 0.5
});
```

### Adjust Camera Position

```javascript
camera.position.set(0.5, 0.5, 0.5);  // x, y, z coordinates
```

### Change Background

```javascript
scene.background = new THREE.Color(0x1a1a1a);  // Dark grey (default)
// Or try: 0xffffff (white), 0x87ceeb (sky blue), etc.
```

### Modify Auto-Rotation Speed

```javascript
controls.autoRotateSpeed = 2.0;  // Higher = faster rotation
```

## Browser Compatibility

Works in all modern browsers with WebGL support:
- Chrome/Edge 90+
- Firefox 88+
- Safari 14+
- Opera 76+

## Troubleshooting

### STL Files Not Loading

1. **CORS Issues**: Make sure you're serving via HTTP server, not `file://`
2. **GitHub URL**: Ensure you're using `raw.githubusercontent.com`, not `github.com`
3. **Path**: Verify the mesh file paths are correct
4. **Console**: Check browser console (F12) for error messages

### Performance Issues

- Reduce shadow quality in renderer settings
- Lower `antialias` setting
- Disable auto-rotation on mobile devices

### Black Screen

- Check browser console for WebGL errors
- Verify Three.js library is loading correctly
- Ensure camera position isn't inside a mesh

## License

This visualizer is part of the Lekiwi ROS 2 project. The STL meshes and URDF are the intellectual property of the project authors.

## Credits

- Built with [Three.js](https://threejs.org/)
- URDF specifications from `lekiwi_description` package
- Robot design: Lekiwi omnidirectional mobile robot
