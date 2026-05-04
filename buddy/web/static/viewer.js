/**
 * Buddy Arm 3D Viewer - Three.js visualization
 *
 * Renders a simplified representation of the 5-DoF arm + gripper using
 * cylinder and box meshes.  Subscribes to the WebSocket state stream
 * (forwarded via app.js) and updates joint rotations in real time.
 *
 * Joint chain (same order as arm.py DEFAULT_JOINT_CONFIGS):
 *   0: base      - rotates around Y axis (yaw)
 *   1: shoulder  - rotates around Z axis in base frame (pitch)
 *   2: elbow     - rotates around Z axis in shoulder frame (pitch)
 *   3: wrist     - rotates around Z axis in elbow frame (pitch)
 *   4: wrist_rot - rotates around Y axis in wrist frame (roll)
 *   5: gripper   - open/close visualization
 *
 * Link lengths match kinematics.py DEFAULT_LINKS (mm), scaled to scene units
 * (1 unit = 1 mm is fine for Three.js with appropriate camera distance).
 */

import * as THREE from "https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js";
import { OrbitControls } from "https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/controls/OrbitControls.js";

// ---- Link dimensions (mm, matching kinematics.py defaults) ------------------

const BASE_HEIGHT   = 100;
const UPPER_ARM     = 120;
const FOREARM       = 120;
const WRIST_OFFSET  = 0;
const TOOL_LENGTH   = 60;

// Visual widths (aesthetic, not physical).
const BASE_RADIUS      = 40;
const BASE_PLATE_H     = 16;
const LINK_RADIUS      = 14;
const JOINT_RADIUS     = 18;
const GRIPPER_WIDTH    = 10;
const GRIPPER_LENGTH   = 40;
const GRIPPER_DEPTH    = 6;
const GRIPPER_GAP_OPEN = 30;

// ---- Colors -----------------------------------------------------------------

const COL_BASE      = 0x4a4a6a;
const COL_LINK      = 0x5a5a8a;
const COL_JOINT     = 0x6c63ff;
const COL_GRIPPER   = 0x8888aa;
const COL_GROUND    = 0x2a2a3a;

// ---- Helpers ----------------------------------------------------------------

function deg2rad(d) { return d * Math.PI / 180; }

function makeCylinder(radius, height, color) {
    var geo = new THREE.CylinderGeometry(radius, radius, height, 16);
    var mat = new THREE.MeshPhongMaterial({ color: color });
    return new THREE.Mesh(geo, mat);
}

function makeSphere(radius, color) {
    var geo = new THREE.SphereGeometry(radius, 16, 12);
    var mat = new THREE.MeshPhongMaterial({ color: color });
    return new THREE.Mesh(geo, mat);
}

function makeBox(w, h, d, color) {
    var geo = new THREE.BoxGeometry(w, h, d);
    var mat = new THREE.MeshPhongMaterial({ color: color });
    return new THREE.Mesh(geo, mat);
}

// ---- Build the arm scene graph ----------------------------------------------

function buildArm() {
    // The hierarchy mirrors the kinematic chain.
    // Each joint group is placed at the joint origin; rotation is applied to
    // the group, and the child link is offset along the local Y axis.

    // Root group (fixed to world).
    var root = new THREE.Group();

    // -- Base platform (fixed) --
    var basePlate = makeCylinder(BASE_RADIUS, BASE_PLATE_H, COL_BASE);
    basePlate.position.y = BASE_PLATE_H / 2;
    root.add(basePlate);

    // -- Base yaw joint (rotates around world Y) --
    var baseJoint = new THREE.Group();
    baseJoint.position.y = BASE_PLATE_H;
    root.add(baseJoint);

    // Base column up to shoulder.
    var baseCol = makeCylinder(LINK_RADIUS, BASE_HEIGHT, COL_LINK);
    baseCol.position.y = BASE_HEIGHT / 2;
    baseJoint.add(baseCol);

    // -- Shoulder joint --
    var shoulderPivot = new THREE.Group();
    shoulderPivot.position.y = BASE_HEIGHT;
    baseJoint.add(shoulderPivot);

    var shoulderBall = makeSphere(JOINT_RADIUS, COL_JOINT);
    shoulderPivot.add(shoulderBall);

    // Upper arm link (extends along local Y after pitch rotation).
    var upperArm = makeCylinder(LINK_RADIUS - 2, UPPER_ARM, COL_LINK);
    upperArm.position.y = UPPER_ARM / 2;
    shoulderPivot.add(upperArm);

    // -- Elbow joint --
    var elbowPivot = new THREE.Group();
    elbowPivot.position.y = UPPER_ARM;
    shoulderPivot.add(elbowPivot);

    var elbowBall = makeSphere(JOINT_RADIUS * 0.85, COL_JOINT);
    elbowPivot.add(elbowBall);

    var forearm = makeCylinder(LINK_RADIUS - 3, FOREARM, COL_LINK);
    forearm.position.y = FOREARM / 2;
    elbowPivot.add(forearm);

    // -- Wrist pitch joint --
    var wristPivot = new THREE.Group();
    wristPivot.position.y = FOREARM;
    elbowPivot.add(wristPivot);

    var wristBall = makeSphere(JOINT_RADIUS * 0.7, COL_JOINT);
    wristPivot.add(wristBall);

    // Tool extension.
    var toolLen = WRIST_OFFSET + TOOL_LENGTH;
    var toolStick = makeCylinder(LINK_RADIUS - 5, toolLen, COL_LINK);
    toolStick.position.y = toolLen / 2;
    wristPivot.add(toolStick);

    // -- Wrist roll joint --
    var rollPivot = new THREE.Group();
    rollPivot.position.y = toolLen;
    wristPivot.add(rollPivot);

    // -- Gripper --
    var gripperGroup = new THREE.Group();
    rollPivot.add(gripperGroup);

    var fingerL = makeBox(GRIPPER_WIDTH, GRIPPER_LENGTH, GRIPPER_DEPTH, COL_GRIPPER);
    fingerL.position.set(-GRIPPER_GAP_OPEN / 2, GRIPPER_LENGTH / 2, 0);
    gripperGroup.add(fingerL);

    var fingerR = makeBox(GRIPPER_WIDTH, GRIPPER_LENGTH, GRIPPER_DEPTH, COL_GRIPPER);
    fingerR.position.set(GRIPPER_GAP_OPEN / 2, GRIPPER_LENGTH / 2, 0);
    gripperGroup.add(fingerR);

    return {
        root: root,
        baseJoint: baseJoint,
        shoulderPivot: shoulderPivot,
        elbowPivot: elbowPivot,
        wristPivot: wristPivot,
        rollPivot: rollPivot,
        gripperGroup: gripperGroup,
        fingerL: fingerL,
        fingerR: fingerR,
    };
}

// ---- Viewer class -----------------------------------------------------------

class BuddyViewer {
    constructor(containerId) {
        this.container = document.getElementById(containerId);
        if (!this.container) return;

        this._animate = this._animate.bind(this);
        this._onResize = this._onResize.bind(this);

        // Defer init until the container has layout dimensions — CSS may
        // not have resolved heights yet on first paint.
        this._waitForSize();
    }

    _waitForSize() {
        var w = this.container.clientWidth;
        var h = this.container.clientHeight;
        if (w > 0 && h > 0) {
            this._initScene();
            this._initArm();
            window.addEventListener("resize", this._onResize);
            this._animate();
        } else {
            requestAnimationFrame(() => this._waitForSize());
        }
    }

    _initScene() {
        var w = this.container.clientWidth;
        var h = this.container.clientHeight;

        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x1a1b2e);

        // Camera.
        this.camera = new THREE.PerspectiveCamera(45, w / h, 1, 5000);
        this.camera.position.set(350, 300, 350);
        this.camera.lookAt(0, 150, 0);

        // Renderer.
        this.renderer = new THREE.WebGLRenderer({ antialias: true });
        this.renderer.setSize(w, h);
        this.renderer.setPixelRatio(window.devicePixelRatio);
        this.container.appendChild(this.renderer.domElement);

        // Orbit controls.
        this.controls = new OrbitControls(this.camera, this.renderer.domElement);
        this.controls.target.set(0, 150, 0);
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.1;
        this.controls.update();

        // Lights.
        var ambient = new THREE.AmbientLight(0xffffff, 0.5);
        this.scene.add(ambient);

        var dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
        dirLight.position.set(200, 400, 300);
        this.scene.add(dirLight);

        var backLight = new THREE.DirectionalLight(0x6c63ff, 0.3);
        backLight.position.set(-200, 200, -200);
        this.scene.add(backLight);

        // Ground grid.
        var grid = new THREE.GridHelper(600, 20, 0x3a3c5e, 0x2d2f4e);
        this.scene.add(grid);

        // Axes helper (small).
        var axes = new THREE.AxesHelper(50);
        this.scene.add(axes);
    }

    _initArm() {
        this.arm = buildArm();
        this.scene.add(this.arm.root);

        // Set initial pose (home = all joints at 0 rotation visually).
        this._setJointAngles([0, 0, 0, 0, 0]);
    }

    _onResize() {
        var w = this.container.clientWidth;
        var h = this.container.clientHeight;
        if (w === 0 || h === 0) return;
        this.camera.aspect = w / h;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(w, h);
    }

    _animate() {
        requestAnimationFrame(this._animate);
        this.controls.update();
        this.renderer.render(this.scene, this.camera);
    }

    /**
     * Set joint rotations from an array of 5 angles in degrees.
     *
     * Convention (matching kinematics.py):
     *   [0] base     - yaw around Y (0 = +X direction)
     *   [1] shoulder - pitch (0 = horizontal, +90 = up)
     *   [2] elbow    - pitch relative to upper arm
     *   [3] wrist    - pitch relative to forearm
     *   [4] wrist_rot - roll around tool axis
     *
     * The arm scene graph uses Y-up, with links extending along +Y.
     * Pitch rotations are about the local Z axis (Three.js Z = lateral).
     */
    _setJointAngles(angles) {
        if (!angles || angles.length < 5) return;

        // Base yaw: rotate around Y.
        this.arm.baseJoint.rotation.y = deg2rad(angles[0]);

        // Shoulder pitch: the link extends along +Y; pitch rotates about Z.
        // kinematics says 0 = horizontal, which in our scene is the link
        // pointing along +Y (already horizontal if base is at ground level).
        // We map shoulder angle to rotation about Z.
        this.arm.shoulderPivot.rotation.z = deg2rad(angles[1]);

        // Elbow pitch.
        this.arm.elbowPivot.rotation.z = deg2rad(angles[2]);

        // Wrist pitch.
        this.arm.wristPivot.rotation.z = deg2rad(angles[3]);

        // Wrist roll: about the tool axis which is local Y.
        this.arm.rollPivot.rotation.y = deg2rad(angles[4]);
    }

    /**
     * Update gripper visualization.
     * @param {number} openFraction 0 = closed, 1 = fully open.
     */
    _setGripper(openFraction) {
        var gap = GRIPPER_GAP_OPEN * openFraction;
        if (gap < 2) gap = 2; // minimum visual gap
        this.arm.fingerL.position.x = -gap / 2;
        this.arm.fingerR.position.x = gap / 2;
    }

    /**
     * Called by app.js whenever a new state arrives from the WebSocket.
     * @param {Object} state - The JSON state payload from the server.
     */
    updateState(state) {
        if (!state || !state.joints) return;

        var angles = [];
        var gripperAngle = null;
        var gripperMin = 0;
        var gripperMax = 180;

        for (var i = 0; i < state.joints.length; i++) {
            var j = state.joints[i];
            if (j.is_gripper) {
                gripperAngle = j.angle;
                gripperMin = j.min_angle;
                gripperMax = j.max_angle;
            } else {
                // Convert from the arm's user-facing degrees to the kinematic
                // convention used by the viewer.  The arm stores absolute angles
                // (typically with 180 as home/center), so we subtract the home
                // position to get a relative rotation for visualization.
                // Home is typically 180 for revolute joints.
                var home = 180;
                var rel = (j.angle !== null) ? j.angle - home : 0;
                angles.push(rel);
            }
        }

        this._setJointAngles(angles);

        if (gripperAngle !== null) {
            var range = gripperMax - gripperMin;
            var frac = range > 0 ? (gripperAngle - gripperMin) / range : 0;
            this._setGripper(frac);
        }
    }
}

// ---- Initialize on load -----------------------------------------------------

var viewer = null;

function initViewer() {
    viewer = new BuddyViewer("viewer-canvas");
    // Expose globally so app.js can forward state updates.
    window.buddyViewer = viewer;
}

if (document.readyState === "loading") {
    document.addEventListener("DOMContentLoaded", initViewer);
} else {
    initViewer();
}
