/**
 * Buddy Arm Web Controller - Control Logic
 *
 * Connects to the backend via REST and WebSocket, manages joint sliders,
 * IK panel, torque/gripper controls, and console output.
 */

// ---- Configuration ----------------------------------------------------------

const WS_RECONNECT_MS = 2000;
const SLIDER_DEBOUNCE_MS = 50;

// ---- State ------------------------------------------------------------------

let ws = null;
let wsConnected = false;
let jointState = null;        // Latest state from WS or GET /state
let sliderDebounceTimers = {};
let torqueEnabled = false;

// ---- DOM References ---------------------------------------------------------

const elWsDot = document.getElementById("ws-dot");
const elWsLabel = document.getElementById("ws-label");
const elSliders = document.getElementById("joint-sliders");
const elConsole = document.getElementById("console-output");

// ---- Console ----------------------------------------------------------------

function logToConsole(msg, cls) {
    const el = document.createElement("div");
    el.className = "log-entry" + (cls ? " log-" + cls : "");
    const ts = new Date().toLocaleTimeString();
    el.textContent = "[" + ts + "] " + msg;
    elConsole.appendChild(el);
    elConsole.scrollTop = elConsole.scrollHeight;
    // Keep at most 200 lines.
    while (elConsole.childElementCount > 200) {
        elConsole.removeChild(elConsole.firstChild);
    }
}

// ---- REST helpers -----------------------------------------------------------

async function apiFetch(path, opts) {
    opts = opts || {};
    try {
        const resp = await fetch(path, opts);
        const data = await resp.json();
        if (!resp.ok || data.ok === false) {
            logToConsole("API " + path + ": " + (data.error || resp.statusText), "error");
        }
        return data;
    } catch (err) {
        logToConsole("API " + path + " failed: " + err.message, "error");
        return null;
    }
}

async function apiPost(path, body) {
    return apiFetch(path, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(body),
    });
}

// ---- WebSocket --------------------------------------------------------------

function wsConnect() {
    if (ws && (ws.readyState === WebSocket.OPEN || ws.readyState === WebSocket.CONNECTING)) {
        return;
    }
    const proto = location.protocol === "https:" ? "wss:" : "ws:";
    const url = proto + "//" + location.host + "/ws";
    ws = new WebSocket(url);

    ws.onopen = function () {
        wsConnected = true;
        elWsDot.classList.add("connected");
        elWsLabel.textContent = "Connected";
        logToConsole("WebSocket connected", "success");
    };

    ws.onmessage = function (evt) {
        try {
            const state = JSON.parse(evt.data);
            jointState = state;
            updateSlidersFromState(state);
            // Forward to 3D viewer if present.
            if (window.buddyViewer && typeof window.buddyViewer.updateState === "function") {
                window.buddyViewer.updateState(state);
            }
        } catch (e) {
            // Ignore parse errors on WS messages.
        }
    };

    ws.onclose = function () {
        wsConnected = false;
        elWsDot.classList.remove("connected");
        elWsLabel.textContent = "Disconnected";
        logToConsole("WebSocket disconnected, reconnecting...", "error");
        setTimeout(wsConnect, WS_RECONNECT_MS);
    };

    ws.onerror = function () {
        // onclose will fire after this.
    };
}

// ---- Joint Sliders ----------------------------------------------------------

/** Build slider DOM for each joint returned by /state. */
function buildSliders(joints) {
    elSliders.innerHTML = "";
    joints.forEach(function (j, idx) {
        if (j.is_gripper) return; // Gripper has its own button.

        var group = document.createElement("div");
        group.className = "joint-slider-group";

        var label = document.createElement("div");
        label.className = "slider-label";

        var nameSpan = document.createElement("span");
        nameSpan.className = "joint-name";
        nameSpan.textContent = j.name;

        var valueSpan = document.createElement("span");
        valueSpan.className = "joint-value";
        valueSpan.id = "val-" + j.name;
        valueSpan.textContent = (j.angle !== null ? j.angle.toFixed(1) : "--") + "°";

        label.appendChild(nameSpan);
        label.appendChild(valueSpan);

        var slider = document.createElement("input");
        slider.type = "range";
        slider.id = "slider-" + j.name;
        slider.min = j.min_angle;
        slider.max = j.max_angle;
        slider.step = 0.5;
        slider.value = j.angle !== null ? j.angle : j.min_angle;
        slider.dataset.jointName = j.name;
        slider.dataset.jointIndex = idx;

        slider.addEventListener("input", onSliderInput);

        group.appendChild(label);
        group.appendChild(slider);
        elSliders.appendChild(group);
    });
}

/** When the user drags a slider, send debounced move command. */
function onSliderInput(evt) {
    var slider = evt.target;
    var name = slider.dataset.jointName;
    var val = parseFloat(slider.value);

    // Update label immediately.
    var valEl = document.getElementById("val-" + name);
    if (valEl) valEl.textContent = val.toFixed(1) + "°";

    // Debounce the actual send.
    if (sliderDebounceTimers[name]) {
        clearTimeout(sliderDebounceTimers[name]);
    }
    sliderDebounceTimers[name] = setTimeout(function () {
        sendJointMove(name, val);
    }, SLIDER_DEBOUNCE_MS);
}

function sendJointMove(name, angle) {
    var angles = {};
    angles[name] = angle;
    apiPost("/move", { angles: angles });
}

/**
 * Update slider positions from WS state, but only if the user is not
 * currently dragging that slider.
 */
function updateSlidersFromState(state) {
    if (!state || !state.joints) return;
    state.joints.forEach(function (j) {
        if (j.is_gripper || j.angle === null) return;
        var slider = document.getElementById("slider-" + j.name);
        if (!slider) return;
        // Don't fight the user if they are actively dragging.
        if (document.activeElement === slider) return;
        slider.value = j.angle;
        var valEl = document.getElementById("val-" + j.name);
        if (valEl) valEl.textContent = j.angle.toFixed(1) + "°";
    });
}

// ---- Torque / Gripper / Home ------------------------------------------------

function onTorqueToggle() {
    torqueEnabled = !torqueEnabled;
    var btn = document.getElementById("btn-torque");
    apiPost("/torque", { enabled: torqueEnabled }).then(function (data) {
        if (data && data.ok) {
            btn.textContent = torqueEnabled ? "Torque ON" : "Torque OFF";
            btn.classList.toggle("active", torqueEnabled);
            logToConsole("Torque " + (torqueEnabled ? "enabled" : "disabled"), "success");
        } else {
            torqueEnabled = !torqueEnabled; // revert
        }
    });
}

function onGripperOpen() {
    apiPost("/gripper", { action: "open" }).then(function (data) {
        if (data && data.ok) logToConsole("Gripper opened", "success");
    });
}

function onGripperClose() {
    apiPost("/gripper", { action: "close" }).then(function (data) {
        if (data && data.ok) logToConsole("Gripper closed", "success");
    });
}

function onHome() {
    apiPost("/home", {}).then(function (data) {
        if (data && data.ok) logToConsole("Moving to home position", "success");
    });
}

// ---- IK Panel ---------------------------------------------------------------

function onIKMove() {
    var x = parseFloat(document.getElementById("ik-x").value) || 0;
    var y = parseFloat(document.getElementById("ik-y").value) || 0;
    var z = parseFloat(document.getElementById("ik-z").value) || 0;
    var pitch = parseFloat(document.getElementById("ik-pitch").value) || 0;
    var roll = parseFloat(document.getElementById("ik-roll").value) || 0;

    logToConsole("IK move: x=" + x + " y=" + y + " z=" + z +
                 " pitch=" + pitch + " roll=" + roll, "info");
    apiPost("/move", {
        pose: { x: x, y: y, z: z, tool_pitch_deg: pitch, tool_roll_deg: roll }
    });
}

// ---- Console Input ----------------------------------------------------------

function onConsoleSend() {
    var input = document.getElementById("console-input");
    var cmd = input.value.trim();
    if (!cmd) return;
    input.value = "";

    logToConsole("> " + cmd, "info");

    // Placeholder: /cli endpoint doesn't exist yet (Phase 7).
    // For now just log a note.
    apiPost("/cli", { command: cmd }).then(function (data) {
        if (data === null) {
            logToConsole("CLI endpoint not available yet (Phase 7)", "error");
        } else if (data.ok) {
            logToConsole(data.result || "OK", "success");
        }
    });
}

function onConsoleKeydown(evt) {
    if (evt.key === "Enter") {
        onConsoleSend();
    }
}

// ---- Initialization ---------------------------------------------------------

async function init() {
    logToConsole("Buddy Arm Controller initializing...", "info");

    // Fetch initial state to build sliders.
    var state = await apiFetch("/state");
    if (state && state.joints) {
        jointState = state;
        buildSliders(state.joints);
        logToConsole("Loaded " + state.joints.length + " joints", "success");
    } else {
        logToConsole("Could not fetch initial state", "error");
    }

    // Wire up buttons.
    document.getElementById("btn-torque").addEventListener("click", onTorqueToggle);
    document.getElementById("btn-gripper-open").addEventListener("click", onGripperOpen);
    document.getElementById("btn-gripper-close").addEventListener("click", onGripperClose);
    document.getElementById("btn-home").addEventListener("click", onHome);
    document.getElementById("btn-ik-move").addEventListener("click", onIKMove);
    document.getElementById("btn-console-send").addEventListener("click", onConsoleSend);
    document.getElementById("console-input").addEventListener("keydown", onConsoleKeydown);

    // Start WebSocket.
    wsConnect();
}

document.addEventListener("DOMContentLoaded", init);
