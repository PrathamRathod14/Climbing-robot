import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js'
import { camera } from './camera'
import { renderer } from './renderer'

export const controls = new OrbitControls(camera, renderer.domElement)
controls.enableDamping = true
controls.enableZoom = true
controls.zoomSpeed = 1.15
controls.rotateSpeed = 0.8
controls.panSpeed = 0.7
controls.minDistance = 2.2
controls.maxDistance = 18
