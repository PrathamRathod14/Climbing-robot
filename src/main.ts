import {
  AmbientLight,
  BoxGeometry,
  BufferGeometry,
  Color,
  CylinderGeometry,
  DirectionalLight,
  Group,
  Line,
  LineBasicMaterial,
  MathUtils,
  Mesh,
  MeshStandardMaterial,
  PlaneGeometry,
  SphereGeometry,
  Vector3,
} from 'three'

import camera from './core/camera'
import { fpsGraph } from './core/gui'
import { controls } from './core/orbit-control'
import { renderer, scene } from './core/renderer'
import './style.css'

type LimbKey = 'frontLeft' | 'frontRight' | 'rearLeft' | 'rearRight'
type SimPhase = 'walkFloor' | 'attach' | 'climb' | 'fall' | 'recover'

interface Hold {
  mesh: Mesh
  point: Vector3
  index: number
  used: boolean
}

interface Limb {
  key: LimbKey
  side: number
  shoulderZ: number
  upper: Mesh
  lower: Mesh
  hip: Mesh
  knee: Mesh
  foot: Mesh
  contact: Hold
  start: Vector3
  target: Vector3
}

interface Climber {
  root: Group
  body: Group
  limbs: Limb[]
}

const stepLabel = document.querySelector('#step-label')
const statusLabel = document.querySelector('#status-label')
const modeLabel = document.querySelector('#mode-label')
const heightValue = document.querySelector('#height-value')
const gaitValue = document.querySelector('#gait-value')
const activeFootValue = document.querySelector('#active-foot-value')
const strainValue = document.querySelector('#strain-value')
const playToggle = document.querySelector<HTMLButtonElement>('#play-toggle')
const stepOnceButton = document.querySelector<HTMLButtonElement>('#step-once')
const resetButton = document.querySelector<HTMLButtonElement>('#reset-sim')
const randomizeButton = document.querySelector<HTMLButtonElement>('#randomize-wall')

const palette = {
  background: '#eef3f0',
  wall: '#d8e1dd',
  rail: '#9badab',
  body: '#f8faf7',
  panel: '#d7dfdc',
  dark: '#263238',
  joint: '#111827',
  foot: '#75b843',
  hold: '#d5bd55',
  active: '#32a6d8',
  route: '#4aa3c7',
}

const config = {
  wallHeight: 16,
  wallWidth: 3.4,
  rowSpacing: 0.36,
  maxLegLength: 0.52,
  comfortableLegLength: 0.46,
  stepDuration: 2.2,
  climbSpeed: 0.75,
  bodyWallGap: -0.5,
  gravityAccel: 9.80665,
  robotMass: 12.0,
  adhesiveShearPerFoot: 42.0,
  adhesiveNormalPerFoot: 70.0,
  wallPeelRatio: 0.32,
  physicsStep: 1 / 120,
  fallDistanceLimit: 0.9,
  demoFallHeights: [0.45, 0.75, 0.95],
  fallDuration: 2.5,
  recoverDuration: 3.8,
  bodySide: 0.13,
  floorFootSide: 0.18,
  kneeSide: 0.035,
  gravityBodySag: 0.055,
  swingBodySag: 0.035,
  gripPeelOffset: 0.018,
}

const sim = {
  running: true,
  manualStep: false,
  phase: 'walkFloor' as SimPhase,
  phaseTime: 0,
  moveProgress: 1,
  stepCount: 0,
  activeLimb: null as Limb | null,
  seed: 0,
  baseHeight: -8.6,
  topHeight: 16.2,
  direction: 1,
  cycleCount: 0,
  message: 'Go2-inspired procedural model: contact planner ready',
  gravityVelY: 0,
  gravityVelZ: 0,
  gripLoad: 0,
  physicsAccumulator: 0,
  fallAttempt: 0,
  fallStart: new Vector3(),
  fallEnd: new Vector3(0, -2.0, -8.22),
}

function narrowWallFoot(_limb: Pick<Limb, 'side' | 'shoulderZ'>, hold: Hold) {
  return hold.point.clone().add(new Vector3(0, -0.18, 0))
}

function makeMaterial(color: string, roughness = 0.55, metalness = 0.05) {
  return new MeshStandardMaterial({
    color: new Color(color),
    roughness,
    metalness,
  })
}

scene.background = new Color(palette.background)
scene.add(new AmbientLight(0xFFFFFF, 1.15))

const keyLight = new DirectionalLight(0xFFFFFF, 2.4)
keyLight.position.set(-4.5, -7.5, 8)
keyLight.castShadow = true
scene.add(keyLight)

const rimLight = new DirectionalLight(0xD9F0FF, 0.9)
rimLight.position.set(4, -5, 3)
scene.add(rimLight)

function makeWall(seed = 0) {
  const group = new Group()
  const holds: Hold[] = []
  const rows = 31
  const firstHoldZ = -8.6
  const lastHoldZ = firstHoldZ + (rows - 1) * config.rowSpacing
  const panelPadding = 1.4
  const panelHeight = lastHoldZ - firstHoldZ + panelPadding * 2
  const panelCenterZ = (firstHoldZ + lastHoldZ) / 2

  const panel = new Mesh(
    new PlaneGeometry(config.wallWidth, panelHeight, 18, 90),
    makeMaterial(palette.wall, 0.84, 0.02),
  )
  panel.rotation.x = Math.PI / 2
  panel.position.set(0, 0.08, panelCenterZ)
  panel.receiveShadow = true
  group.add(panel)

  const railMaterial = makeMaterial(palette.rail, 0.66, 0.08)
  for (const x of [-config.wallWidth / 2, config.wallWidth / 2]) {
    const rail = new Mesh(new BoxGeometry(0.1, 0.12, panelHeight), railMaterial)
    rail.position.set(x, 0.0, panelCenterZ)
    rail.castShadow = true
    group.add(rail)
  }

  const columns = [-0.9, -0.45, 0, 0.45, 0.9]
  const routePoints: Vector3[] = []

  for (let row = 0; row < rows; row++) {
    const z = firstHoldZ + row * config.rowSpacing
    const corridor = Math.sin(row * 0.38 + seed) * 0.42
    const rowShift = Math.sin(row * 0.6 + seed) * 0.05
    const rowColumns = columns.filter((baseX, columnIndex) => {
      const nearPath = Math.abs(baseX - corridor) < 0.72
      const supportPair = columnIndex === 1 || columnIndex === 3
      const gap = row > 6 && columnIndex !== 2 && (row + columnIndex * 3 + Math.floor(seed * 10)) % 11 === 0
      return (nearPath || supportPair) && !gap
    })

    const safeColumns = rowColumns.length >= 4 ? rowColumns : columns.slice(0, 5)
    let bestRoutePoint: Vector3 | null = null
    let bestRouteDistance = Number.POSITIVE_INFINITY

    for (const baseX of safeColumns) {
      const x = baseX + rowShift + Math.sin(row * 1.4 + baseX + seed) * 0.045
      const radius = row % 5 === 0 ? 0.15 : 0.115
      const hold = new Mesh(
        new CylinderGeometry(radius, radius * 0.76, 0.16, 24),
        makeMaterial(palette.hold, 0.62, 0.04),
      )
      hold.rotation.x = Math.PI / 2
      hold.position.set(x, -0.08, z)
      hold.castShadow = true
      group.add(hold)
      holds.push({ mesh: hold, point: hold.position.clone(), index: holds.length, used: false })

      const routeDistance = Math.abs(x - corridor)
      if (routeDistance < bestRouteDistance) {
        bestRouteDistance = routeDistance
        bestRoutePoint = hold.position.clone().add(new Vector3(0, -0.05, 0))
      }
    }

    if (bestRoutePoint && row % 2 === 0) {
      routePoints.push(bestRoutePoint)
    }
  }

  const routeLine = new Line(
    new BufferGeometry().setFromPoints(routePoints),
    new LineBasicMaterial({ color: palette.route, transparent: true, opacity: 0.16 }),
  )
  group.add(routeLine)

  return { group, holds }
}

function startingHolds(holds: Hold[]) {
  const bottom = holds.filter(hold => hold.point.z < -7.3)
  const used = new Set<Hold>()
  const pick = (x: number, z: number) => {
    const hold = bottom
      .filter(candidate => !used.has(candidate))
      .sort((a, b) => {
        const aScore = Math.abs(a.point.x - x) * 2 + Math.abs(a.point.z - z)
        const bScore = Math.abs(b.point.x - x) * 2 + Math.abs(b.point.z - z)
        return aScore - bScore
      })[0] ?? bottom.find(candidate => !used.has(candidate)) ?? bottom[0]
    used.add(hold)
    return hold
  }
  return [
    pick(-0.36, -7.75),
    pick(0.36, -7.75),
    pick(-0.18, -8.35),
    pick(0.22, -8.35),
  ]
}

function topReached() {
  return climber.body.position.z - sim.baseHeight >= 9.7
}


function addBodyPart(group: Group, mesh: Mesh, position: Vector3) {
  mesh.position.copy(position)
  mesh.castShadow = true
  mesh.receiveShadow = true
  group.add(mesh)
}

function makeGo2InspiredRobot(startHolds: Hold[]): Climber {
  const root = new Group()
  const body = new Group()
  root.add(body)

  const bodyMaterial = makeMaterial(palette.body, 0.36, 0.18)
  const panelMaterial = makeMaterial(palette.panel, 0.42, 0.1)
  const darkMaterial = makeMaterial(palette.dark, 0.48, 0.25)
  const legMaterial = makeMaterial(palette.joint, 0.48, 0.26)
  const footMaterial = makeMaterial(palette.foot, 0.38, 0.08)

  addBodyPart(body, new Mesh(new BoxGeometry(0.58, 0.28, 0.78), bodyMaterial), new Vector3(0, 0, 0))
  addBodyPart(body, new Mesh(new BoxGeometry(0.4, 0.13, 0.22), panelMaterial), new Vector3(0, -0.17, 0.08))
  addBodyPart(body, new Mesh(new BoxGeometry(0.28, 0.16, 0.18), darkMaterial), new Vector3(0, -0.12, 0.5))

  const lens = new Mesh(new SphereGeometry(0.065, 18, 12), makeMaterial('#6ddcff', 0.2, 0.2))
  lens.position.set(0, -0.22, 0.5)
  body.add(lens)

  const hipOffsets: Array<[LimbKey, number, number, Hold]> = [
    ['frontLeft', -1, 0.3, startHolds[0]],
    ['frontRight', 1, 0.3, startHolds[1]],
    ['rearLeft', -1, -0.3, startHolds[2]],
    ['rearRight', 1, -0.3, startHolds[3]],
  ]

  const limbs: Limb[] = hipOffsets.map(([key, side, shoulderZ, hold]) => {
    const upper: Mesh = new Mesh(new CylinderGeometry(0.026, 0.034, 1, 16), legMaterial)
    const lower: Mesh = new Mesh(new CylinderGeometry(0.024, 0.03, 1, 16), legMaterial)
    const hip: Mesh = new Mesh(new SphereGeometry(0.056, 16, 12), darkMaterial)
    const knee: Mesh = new Mesh(new SphereGeometry(0.045, 16, 12), darkMaterial)
    const foot: Mesh = new Mesh(new BoxGeometry(0.16, 0.06, 0.1), footMaterial)
    for (const part of [upper, lower, hip, knee, foot]) {
      part.castShadow = true
      root.add(part)
    }
    const target = narrowWallFoot({ side, shoulderZ }, hold)
    hold.used = true
    return { key, side, shoulderZ, upper, lower, hip, knee, foot, contact: hold, start: target.clone(), target }
  })

  return { root, body, limbs }
}

function setCylinderBetween(mesh: Mesh, start: Vector3, end: Vector3) {
  const direction = end.clone().sub(start)
  const midpoint = start.clone().add(end).multiplyScalar(0.5)
  mesh.position.copy(midpoint)
  mesh.scale.set(1, Math.max(direction.length(), 0.001), 1)
  mesh.quaternion.setFromUnitVectors(new Vector3(0, 1, 0), direction.normalize())
}

let wall = makeWall()
scene.add(wall.group)

const floorZ = -9.45
const floor = new Mesh(
  new PlaneGeometry(8, 5.6, 12, 8),
  makeMaterial('#f8fbf9', 0.82, 0),
)
floor.position.set(0, -2.1, floorZ)
floor.receiveShadow = true
scene.add(floor)

const wallBase = new Mesh(
  new BoxGeometry(config.wallWidth + 0.55, 0.16, 0.12),
  makeMaterial('#9badab', 0.7, 0.05),
)
wallBase.position.set(0, -0.02, floorZ + 0.12)
wallBase.castShadow = true
scene.add(wallBase)

let climber = makeGo2InspiredRobot(startingHolds(wall.holds))
scene.add(climber.root)

camera.position.set(0, -8.4, -7.35)
camera.lookAt(0, -0.2, -7.35)
camera.near = 0.1
camera.far = 80
camera.updateProjectionMatrix()
controls.target.set(0, -0.2, -7.35)
controls.enableDamping = true
controls.update()

function contactCenter(skip?: Limb) {
  const contacts = climber.limbs.filter(limb => limb !== skip).map(limb => limb.target)
  return contacts.reduce((sum, point) => sum.add(point), new Vector3()).multiplyScalar(1 / contacts.length)
}

function releaseContact(limb: Limb) {
  limb.contact.used = climber.limbs.some(other => other !== limb && other.contact === limb.contact)
}

function shoulderWorld(limb: Limb) {
  return new Vector3(limb.side * config.bodySide, -0.03, limb.shoulderZ).applyMatrix4(climber.body.matrixWorld)
}

function footPosition(limb: Limb) {
  if (sim.activeLimb === limb && sim.moveProgress < 1) {
    const eased = MathUtils.smoothstep(sim.moveProgress, 0, 1)
    const arc = Math.sin(eased * Math.PI)
    return limb.start
      .clone()
      .lerp(limb.target, eased)
      .add(new Vector3(0, -0.12 * arc, 0))
  }

  return limb.target.clone()
}

function limbStrain(limb: Limb) {
  return shoulderWorld(limb).distanceTo(footPosition(limb)) / config.comfortableLegLength
}

function desiredBodyFromContacts() {
  const swinging = sim.activeLimb && sim.moveProgress > 0 && sim.moveProgress < 1
  const swingArc = swinging ? Math.sin(MathUtils.smoothstep(sim.moveProgress, 0, 1) * Math.PI) : 0
  const target = climber.limbs
    .reduce((sum, limb) => {
      sum.x += limb.target.x - limb.side * config.bodySide
      sum.z += limb.target.z - limb.shoulderZ
      return sum
    }, new Vector3(0, config.bodyWallGap, 0))
    .multiplyScalar(1 / climber.limbs.length)
  target.y = config.bodyWallGap - config.gripPeelOffset * (0.35 + swingArc * 0.65)
  target.z -= config.gravityBodySag + config.swingBodySag * swingArc
  return target
}

function desiredBodyFromWallContacts() {
  const target = climber.limbs
    .reduce((sum, limb) => {
      const foot = narrowWallFoot(limb, limb.contact)
      sum.x += foot.x - limb.side * config.bodySide
      sum.z += foot.z - limb.shoulderZ
      return sum
    }, new Vector3(0, config.bodyWallGap, 0))
    .multiplyScalar(1 / climber.limbs.length)
  target.y = config.bodyWallGap - config.gripPeelOffset * 0.35
  target.z -= config.gravityBodySag
  return target
}

function floorBodyAt(progress: number) {
  const startBody = new Vector3(-0.45, -4.25, floorZ + 0.34)
  const approachBody = new Vector3(0, -1.25, floorZ + 0.34)
  return startBody.lerp(approachBody, progress)
}

function floorFootFor(limb: Limb, bodyPos: Vector3, time = 0, moving = false) {
  const diagonalPhase = limb.key === 'frontLeft' || limb.key === 'rearRight' ? 0 : Math.PI
  const cycle = moving ? Math.sin(time * 4.4 + diagonalPhase) : 0
  const lift = Math.max(0, cycle) * 0.07
  const stride = moving ? Math.sin(time * 4.4 + diagonalPhase) * 0.11 : 0
  const foreAft = limb.shoulderZ
  return new Vector3(
    bodyPos.x + limb.side * config.floorFootSide,
    bodyPos.y + foreAft * 0.42 + stride,
    floorZ + 0.04 + lift,
  )
}

function setAttachPose(progress: number) {
  const approachBody = floorBodyAt(1)
  const wallBody = desiredBodyFromWallContacts()
  const bodyProgress = MathUtils.smoothstep(MathUtils.clamp((progress - 0.15) / 0.85, 0, 1), 0, 1)
  const bodyPos = approachBody.clone().lerp(wallBody, bodyProgress)
  climber.body.position.copy(bodyPos)
  const floorTilt = -Math.PI / 2
  climber.body.rotation.set(MathUtils.lerp(floorTilt, 0, bodyProgress), 0, 0)

  const attachOrder: LimbKey[] = ['frontLeft', 'frontRight', 'rearLeft', 'rearRight']
  for (const limb of climber.limbs) {
    const orderIndex = attachOrder.indexOf(limb.key)
    const footProgress = MathUtils.smoothstep(MathUtils.clamp((progress - orderIndex * 0.18) / 0.38, 0, 1), 0, 1)
    const floorFoot = floorFootFor(limb, approachBody)
    const wallFoot = narrowWallFoot(limb, limb.contact)
    const arc = Math.sin(footProgress * Math.PI) * 0.16
    const footTarget = floorFoot.lerp(wallFoot, footProgress).add(new Vector3(0, -arc * 0.25, arc))
    limb.start.copy(footTarget)
    limb.target.copy(footTarget)
  }
}

function setFloorWalkPose(progress: number, time: number) {
  const bodyPos = floorBodyAt(progress)
  climber.body.position.copy(bodyPos)
  const sway = Math.sin(time * 2.3) * 0.025
  climber.body.rotation.set(-Math.PI / 2, 0, sway)

  for (const limb of climber.limbs) {
    const foot = floorFootFor(limb, bodyPos, time, true)
    limb.start.copy(foot)
    limb.target.copy(foot)
  }
}

function setFallenPose(progress: number) {
  const eased = MathUtils.smoothstep(progress, 0, 1)
  const bodyPos = sim.fallStart.clone().lerp(sim.fallEnd, eased)
  bodyPos.z += Math.sin(eased * Math.PI) * 0.24
  climber.body.position.copy(bodyPos)
  climber.body.rotation.set(
    MathUtils.lerp(0, -Math.PI / 2, eased),
    MathUtils.lerp(0, 0.16, eased),
    Math.sin(eased * Math.PI) * 0.34,
  )

  for (const limb of climber.limbs) {
    const settled = floorFootFor(limb, sim.fallEnd, 0, false)
    limb.start.lerp(settled, eased)
    limb.target.copy(limb.start)
  }
}

function prepareRetryClimb() {
  const starts = startingHolds(wall.holds)
  for (const [index, limb] of climber.limbs.entries()) {
    limb.contact = starts[index]
    limb.contact.used = true
    const foot = floorFootFor(limb, sim.fallEnd, 0, false)
    limb.start.copy(foot)
    limb.target.copy(foot)
  }

  sim.activeLimb = null
  sim.moveProgress = 1
  sim.direction = 1
  sim.gravityVelY = 0
  sim.gravityVelZ = 0
  sim.gripLoad = 0
  sim.physicsAccumulator = 0
  setPhase('recover', `Recovered on floor; attempt ${sim.fallAttempt + 1} will climb higher`)
}

function setPhase(nextPhase: SimPhase, message: string) {
  sim.phase = nextPhase
  sim.phaseTime = 0
  sim.message = message
}

function chooseNextMove() {
  if (sim.moveProgress < 1) {
    return
  }
  if (sim.phase !== 'climb') {
    return
  }

  const gait: LimbKey[] = ['frontLeft', 'rearRight', 'frontRight', 'rearLeft']
  const strainedLimbs = climber.limbs
    .map(limb => ({ limb, strain: limbStrain(limb) }))
    .filter(item => item.strain > 0.65)
    .sort((a, b) => b.strain - a.strain)
    .map(item => item.limb)

  const gaitLimbs = gait
    .slice(sim.stepCount % gait.length)
    .concat(gait.slice(0, sim.stepCount % gait.length))
    .map(key => climber.limbs.find(limb => limb.key === key))
    .filter((limb): limb is Limb => Boolean(limb))

  const orderedLimbs = [
    ...strainedLimbs,
    ...gaitLimbs.filter(limb => !strainedLimbs.includes(limb)),
  ]

  const makePlans = (allowRecovery: boolean) => orderedLimbs.flatMap((limb, orderIndex) => {
    const shoulder = shoulderWorld(limb)
    const lane = limb.side < 0 ? 0.28 : 0.30
    const bodyDesired = climber.body.position.clone().add(new Vector3(limb.side * lane, 0.34, limb.shoulderZ + 0.28 * sim.direction))
    const desired = new Vector3(bodyDesired.x, -0.08, bodyDesired.z)
    return wall.holds
      .filter(hold => hold === limb.contact || !climber.limbs.some(other => other !== limb && other.contact === hold))
      .map((hold) => {
        const holdFoot = narrowWallFoot(limb, hold)
        const reach = shoulder.distanceTo(holdFoot)
        const progress = (hold.point.z - limb.target.z) * sim.direction
        const laneDistance = Math.abs(hold.point.x - limb.side * lane)
        const lateral = Math.abs(hold.point.x - desired.x)
        const vertical = Math.abs(hold.point.z - desired.z)
        const support = contactCenter(limb)
        const supportWidth = Math.abs(hold.point.x - support.x)
        const xEnvelope = Math.abs(holdFoot.x - shoulder.x)
        const zEnvelope = Math.abs(holdFoot.z - shoulder.z)
        const minProgress = allowRecovery ? -config.rowSpacing * 0.45 : 0.015
        const maxProgress = allowRecovery ? config.rowSpacing * 1.3 : config.rowSpacing * 1.2
        const legal = reach <= config.maxLegLength
          && progress > minProgress
          && progress < maxProgress
          && supportWidth < 1.2
          && xEnvelope < 0.44
          && zEnvelope < 0.44
          && laneDistance < 0.34
        const strain = limbStrain(limb)
        const forwardPreference = allowRecovery ? Math.abs(progress - 0.14) : Math.max(0, 0.28 - progress)
        const score = orderIndex * 0.28 + lateral * 1.2 + laneDistance * 5.0 + vertical * 1.8 + forwardPreference * 2.0 + supportWidth * 0.2 - strain * 0.4
        return { limb, hold, reach, legal, score, strain }
      })
  })

  let selected = makePlans(false)
    .filter(plan => plan.legal)
    .sort((a, b) => a.score - b.score)[0]

  if (!selected) {
    selected = makePlans(true)
      .filter(plan => plan.legal)
      .sort((a, b) => a.score - b.score)[0]
  }

  if (!selected) {
    sim.message = sim.direction > 0
      ? 'No legal higher hold inside reach/support limits'
      : 'No legal return hold inside reach/support limits'
    return
  }

  releaseContact(selected.limb)
  selected.limb.contact = selected.hold
  selected.limb.contact.used = true
  selected.limb.start.copy(selected.limb.target)
  selected.limb.target.copy(narrowWallFoot(selected.limb, selected.hold))
  sim.activeLimb = selected.limb
  sim.moveProgress = 0
  sim.stepCount += 1
  sim.message = `${selected.limb.key} ${sim.direction > 0 ? 'climb' : 'return'} hold ${selected.hold.index}; leg ${selected.reach.toFixed(2)}m; strain ${selected.strain.toFixed(2)}`
}

// Gravity uses SI units: kg, meters, seconds, and Newtons. In this scene Z is
// vertical, and the wall is near Y = 0, so falling away from the wall is -Y.
function computeGravity(dt: number) {
  if (sim.phase !== 'climb') {
    sim.gravityVelY = 0
    sim.gravityVelZ = 0
    sim.gripLoad = 0
    return
  }

  const attachedFeet = climber.limbs.filter(limb => limb !== sim.activeLimb || sim.moveProgress >= 1).length
  const weightForce = config.robotMass * config.gravityAccel
  const shearCapacity = attachedFeet * config.adhesiveShearPerFoot
  const normalCapacity = attachedFeet * config.adhesiveNormalPerFoot
  const peelForce = weightForce * config.wallPeelRatio
  const shearLoad = weightForce / Math.max(shearCapacity, 1)
  const normalLoad = peelForce / Math.max(normalCapacity, 1)
  sim.gripLoad = MathUtils.clamp(Math.max(shearLoad, normalLoad), 0, 1)

  const downForce = Math.max(0, weightForce - shearCapacity)
  sim.gravityVelZ -= (downForce / config.robotMass) * dt

  if (normalCapacity >= peelForce) {
    const yError = climber.body.position.y - config.bodyWallGap
    sim.gravityVelY += (-80 * yError - 14 * sim.gravityVelY) * dt
  }
  else {
    const peelAcceleration = (peelForce - normalCapacity) / config.robotMass
    sim.gravityVelY -= peelAcceleration * dt
  }

  climber.body.position.y += sim.gravityVelY * dt
  climber.body.position.z += sim.gravityVelZ * dt

  if (climber.body.position.y > config.bodyWallGap + 0.01) {
    climber.body.position.y = config.bodyWallGap + 0.01
    sim.gravityVelY = 0
  }

  if (climber.body.position.y < config.bodyWallGap - config.fallDistanceLimit) {
    sim.running = false
    sim.message = 'Grip failure: gravity pulled the robot away from the wall'
  }
}

function updateGravity(dt: number) {
  if (!sim.running || sim.phase !== 'climb') {
    sim.physicsAccumulator = 0
    computeGravity(0)
    return
  }

  sim.physicsAccumulator = Math.min(sim.physicsAccumulator + dt, 0.2)
  while (sim.physicsAccumulator >= config.physicsStep) {
    computeGravity(config.physicsStep)
    sim.physicsAccumulator -= config.physicsStep
  }
}

function renderRobot(dt: number) {
  sim.phaseTime += sim.running ? dt : 0

  if (sim.phase === 'walkFloor') {
    const progress = MathUtils.smoothstep(Math.min(sim.phaseTime / 4.8, 1), 0, 1)
    setFloorWalkPose(progress, sim.phaseTime)
    if (sim.running && progress >= 1) {
      setPhase('attach', 'At wall base; placing feet on first holds')
    }
  }
  else if (sim.phase === 'attach') {
    const attachDuration = 2.6
    const progress = MathUtils.smoothstep(Math.min(sim.phaseTime / attachDuration, 1), 0, 1)
    setAttachPose(progress)
    if (sim.phaseTime >= attachDuration) {
      for (const limb of climber.limbs) {
        limb.target.copy(narrowWallFoot(limb, limb.contact))
        limb.start.copy(limb.target)
      }
      climber.body.position.copy(desiredBodyFromWallContacts())
      sim.baseHeight = climber.body.position.z
      sim.direction = 1
      setPhase('climb', 'Wall contact established; starting climb')
    }
  }
  else if (sim.phase === 'fall') {
    const progress = Math.min(sim.phaseTime / config.fallDuration, 1)
    setFallenPose(progress)
    if (progress >= 1) {
      prepareRetryClimb()
    }
  }
  else if (sim.phase === 'recover') {
    const progress = MathUtils.smoothstep(Math.min(sim.phaseTime / config.recoverDuration, 1), 0, 1)
    const bodyPos = sim.fallEnd.clone().lerp(floorBodyAt(1), progress)
    climber.body.position.copy(bodyPos)
    climber.body.rotation.set(-Math.PI / 2, 0, Math.sin(sim.phaseTime * 3) * 0.02)
    for (const limb of climber.limbs) {
      const foot = floorFootFor(limb, bodyPos, sim.phaseTime, true)
      limb.start.copy(foot)
      limb.target.copy(foot)
    }
    if (progress >= 1) {
      setPhase('attach', sim.fallAttempt < config.demoFallHeights.length
        ? `Back at wall base; next release target is ${config.demoFallHeights[sim.fallAttempt].toFixed(1)}m`
        : 'Back at wall base; final climb will finish the route')
    }
  }
  else if (sim.phase === 'climb' && topReached() && sim.moveProgress >= 1) {
    sim.running = false
    sim.message = 'Top reached after recovery climb; robot holding position'
  }

  const climbHeight = climber.body.position.z - sim.baseHeight
  const nextFallHeight = config.demoFallHeights[sim.fallAttempt]
  if (sim.phase === 'climb' && nextFallHeight !== undefined && climbHeight >= nextFallHeight && sim.moveProgress >= 1) {
    sim.fallStart.copy(climber.body.position)
    sim.fallAttempt += 1
    sim.message = `Gravity test ${sim.fallAttempt}/${config.demoFallHeights.length}: release at ${climbHeight.toFixed(1)}m`
    setPhase('fall', sim.message)
  }

  if (sim.phase === 'climb' && sim.gripLoad > 0.92) {
    sim.message = `WARNING: gecko grip at ${(sim.gripLoad * 100).toFixed(0)}% load from gravity`
  }

  if (sim.running && sim.moveProgress >= 1 && sim.phase === 'climb') {
    chooseNextMove()
  }

  if (sim.moveProgress < 1) {
    sim.moveProgress = Math.min(1, sim.moveProgress + (dt / config.stepDuration) * config.climbSpeed)
    if (sim.moveProgress >= 1 && sim.activeLimb) {
      sim.message = `${sim.activeLimb.key} attached; body rebalanced on four contacts`
      sim.activeLimb = null
      if (sim.manualStep) {
        sim.running = false
        sim.manualStep = false
      }
    }
  }

  if (sim.phase === 'climb') {
    const maxStrain = climber.limbs.reduce((max, limb) => Math.max(max, limbStrain(limb)), 0)
    const bodyBlend = MathUtils.clamp(dt * (maxStrain > 1.0 ? 6.0 : 3.0), 0, 0.5)
    const targetBody = desiredBodyFromContacts()
    climber.body.position.lerp(targetBody, bodyBlend)

    const leftZ = climber.limbs.filter(limb => limb.side < 0).reduce((sum, limb) => sum + limb.target.z, 0) / 2
    const rightZ = climber.limbs.filter(limb => limb.side > 0).reduce((sum, limb) => sum + limb.target.z, 0) / 2
    climber.body.rotation.z = MathUtils.lerp(climber.body.rotation.z, MathUtils.clamp((leftZ - rightZ) * 0.1, -0.22, 0.22), MathUtils.clamp(dt * 4.0, 0, 0.5))

    updateGravity(dt)
  }

  for (const limb of climber.limbs) {
    const moving = sim.activeLimb === limb && sim.moveProgress < 1
    const currentFoot = footPosition(limb)
    const shoulder = shoulderWorld(limb)
    const knee = shoulder.clone().lerp(currentFoot, 0.5).add(new Vector3(limb.side * config.kneeSide, -0.14, moving ? 0.06 : 0.02))

    limb.hip.position.copy(shoulder)
    limb.knee.position.copy(knee)
    limb.foot.position.copy(currentFoot)
    limb.foot.rotation.z = limb.side * 0.08
    setCylinderBetween(limb.upper, shoulder, knee)
    setCylinderBetween(limb.lower, knee, currentFoot)
  }
}

function updateWallDisplay() {
  for (const hold of wall.holds) {
    const mat = hold.mesh.material as MeshStandardMaterial
    const active = climber.limbs.some(limb => limb.contact === hold)
    mat.emissive.set(active ? palette.active : '#000000')
    mat.emissiveIntensity = active ? 0.55 : 0
  }
}

function updateCamera() {
  const z = climber.body.position.z
  const target = new Vector3(0, -0.05, z + 0.1)
  controls.target.lerp(target, 0.04)
}

function updateHud() {
  const climbHeight = Math.max(0, climber.body.position.z - sim.baseHeight)
  const highestStrain = climber.limbs.reduce((max, limb) => Math.max(max, limbStrain(limb)), 0)
  const activeFoot = sim.activeLimb?.key ?? 'Stable'

  if (stepLabel) {
    const phaseText: Record<SimPhase, string> = {
      walkFloor: 'Walking to wall',
      attach: 'Attaching',
      climb: 'Climbing',
      fall: 'Falling',
      recover: 'Recovering',
    }
    stepLabel.textContent = `${sim.running ? phaseText[sim.phase] : 'Paused'} | gait step ${sim.stepCount} | height ${climbHeight.toFixed(1)}m`
  }
  if (statusLabel) {
    statusLabel.textContent = sim.message
  }
  if (modeLabel) {
    const phaseLabel: Record<SimPhase, string> = {
      walkFloor: 'Floor Walk',
      attach: 'Attach Wall',
      climb: 'Climb Route',
      fall: 'Gravity Fall',
      recover: 'Retry Setup',
    }
    modeLabel.textContent = sim.running ? phaseLabel[sim.phase] : 'Paused'
  }
  if (heightValue) {
    heightValue.textContent = `${climbHeight.toFixed(1)}m`
  }
  if (gaitValue) {
    gaitValue.textContent = `${sim.stepCount}`
  }
  if (activeFootValue) {
    activeFootValue.textContent = activeFoot
  }
  if (strainValue) {
    const gripPct = (sim.gripLoad * 100).toFixed(0)
    strainValue.textContent = `${highestStrain.toFixed(2)} (gravity: ${gripPct}%)`
  }
  if (playToggle) {
    playToggle.textContent = sim.running ? 'Pause' : 'Play'
  }
}

function resetSimulation(newWall = false) {
  if (newWall) {
    scene.remove(wall.group)
    sim.seed = Math.random() * Math.PI * 2
    wall = makeWall(sim.seed)
    scene.add(wall.group)
  }

  scene.remove(climber.root)
  for (const hold of wall.holds) {
    hold.used = false
  }
  climber = makeGo2InspiredRobot(startingHolds(wall.holds))
  scene.add(climber.root)
  climber.body.position.copy(desiredBodyFromWallContacts())
  sim.baseHeight = climber.body.position.z
  sim.topHeight = sim.baseHeight + 11.2
  sim.direction = 1
  sim.phase = 'walkFloor'
  sim.phaseTime = 0
  sim.running = true
  sim.manualStep = false
  sim.moveProgress = 1
  sim.stepCount = 0
  sim.activeLimb = null
  sim.gravityVelY = 0
  sim.gravityVelZ = 0
  sim.gripLoad = 0
  sim.physicsAccumulator = 0
  sim.fallAttempt = 0
  sim.fallStart.set(0, 0, 0)
  sim.message = newWall
    ? 'New wall generated; quadruped contact planner reset'
    : 'Reset: robot staged on floor'
  setFloorWalkPose(0, 0)
}

playToggle?.addEventListener('click', () => {
  sim.running = !sim.running
  sim.manualStep = false
  updateHud()
})

stepOnceButton?.addEventListener('click', () => {
  if (sim.moveProgress >= 1) {
    sim.running = true
    sim.manualStep = true
    chooseNextMove()
  }
})

resetButton?.addEventListener('click', () => resetSimulation(false))
randomizeButton?.addEventListener('click', () => resetSimulation(true))

resetSimulation(false)

let previous = performance.now()
function loop(now = performance.now()) {
  fpsGraph.begin()
  const dt = Math.min((now - previous) / 1000, 0.05)
  previous = now

  renderRobot(dt)
  updateWallDisplay()
  updateCamera()
  updateHud()
  controls.update()
  renderer.render(scene, camera)
  fpsGraph.end()
  requestAnimationFrame(loop)
}

loop()
