# tremor detection

Detection runs on-device now, not in this web app — see
`tremor_glove/tremor_detect.ino` in the repo root for the actual
dual-channel (gyro+accel), 3-gate, adaptively-calibrated detector (a
direct port of what used to live in `js/tremor.js`). This app's
`js/tremor.js` just displays whatever the firmware already decided,
parsed out of the BLE packet by `js/main.js`.

# 3D asset attribution

The hand model rendered in the web app viewport (webapp/js/scene.js,
webapp/assets/models/RightHand.obj) is "Low Poly Right Hand" by Raziq
Brown, sourced from Poly Pizza (https://poly.pizza/m/cl8ax5B13cp),
licensed under Creative Commons Attribution 3.0
(https://creativecommons.org/licenses/by/3.0/). No changes to geometry
beyond recentering/rescaling for display; material/color is applied by
the app, not part of the original asset.
