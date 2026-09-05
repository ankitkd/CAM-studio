(() => {
  const canvas = document.getElementById('sphereCanvas');
  const ctx = canvas.getContext('2d');
  const viewport = document.getElementById('viewport');
  const fields = Object.fromEntries([
    'designName','sphereRadius','smoothing','sampleCount','finalLegLength','pivotDistance','armLength',
    'ballDiameter','printClearance','edgeMargin','pointCount','sphereError','loopStatus','smoothValue',
    'message','liveScale','runtimeBanner','mechanismSummary','drawPanel','mechanismPanel',
    'drawActions','mechanismActions','stepOneBadge','stepTwoBadge','motionEnvelope','previewLegend','gaitEnvelopeBadge',
    'applyState','phaseValue','phaseSlider','designState','designStateDot','legRule','orientationRule','pivotRule','camEstimate'
  ].map(id => [id, document.getElementById(id)]));
  const buttons = Object.fromEntries([
    'drawTool','orbitTool','undoButton','closeButton','clearButton','downloadButton','downloadTxtButton',
    'continueButton','backButton','generateButton','revealButton','applyButton','playButton','faceViewButton'
  ].map(id => [id, document.getElementById(id)]));

  let path = [];
  let closed = false;
  let mode = 'draw';
  let dragging = false;
  // Start on the outward (-X) side of the follower sphere. Curves drawn here
  // keep the contact ball on the cam side of the straight follower leg.
  let yaw = Math.PI / 2;
  let pitch = 0;
  let previousPointer = null;
  let stage = 'draw';
  let serverReady = false;
  let mechanismLengthWasEdited = false;
  let mechanismDirty = false;
  let appliedMechanism = null;
  let appliedAnalysis = null;
  let animationPlaying = false;
  let animationPhase = 0;
  let previousAnimationTime = 0;
  let lastGeneratedName = null;
  let geometry = {width: 1, height: 1, cx: 0, cy: 0, radius: 1};
  const DRAFT_KEY = 'camstudio.design.v1';
  const DRAFT_FIELDS = [
    'designName', 'sphereRadius', 'smoothing', 'sampleCount', 'finalLegLength',
    'pivotDistance', 'armLength', 'ballDiameter', 'printClearance', 'edgeMargin'
  ];

  const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
  const numeric = (field, fallback) => {
    const value = Number(field.value);
    return Number.isFinite(value) ? value : fallback;
  };
  const normalize = v => {
    const n = Math.hypot(v[0], v[1], v[2]) || 1;
    return [v[0] / n, v[1] / n, v[2] / n];
  };
  const worldToCamera = v => {
    const cy = Math.cos(yaw), sy = Math.sin(yaw), cp = Math.cos(pitch), sp = Math.sin(pitch);
    const x = cy * v[0] + sy * v[2];
    const z = -sy * v[0] + cy * v[2];
    return [x, cp * v[1] - sp * z, sp * v[1] + cp * z];
  };
  const cameraToWorld = v => {
    const cy = Math.cos(yaw), sy = Math.sin(yaw), cp = Math.cos(pitch), sp = Math.sin(pitch);
    const y = cp * v[1] + sp * v[2];
    const z1 = -sp * v[1] + cp * v[2];
    return normalize([cy * v[0] - sy * z1, y, sy * v[0] + cy * z1]);
  };
  const project = v => {
    const c = worldToCamera(v);
    return [geometry.cx + c[0] * geometry.radius, geometry.cy - c[1] * geometry.radius, c[2]];
  };

  function saveDraft() {
    try {
      const values = Object.fromEntries(DRAFT_FIELDS.map(id => [id, fields[id].value]));
      localStorage.setItem(DRAFT_KEY, JSON.stringify({version: 1, path, closed, values}));
    } catch (_) {
      // Private browsing or a locked-down browser may disable local storage.
    }
  }

  function restoreDraft() {
    try {
      const saved = JSON.parse(localStorage.getItem(DRAFT_KEY) || 'null');
      if (!saved || saved.version !== 1 || !Array.isArray(saved.path) || saved.path.length > 20000) return false;
      const validPath = saved.path.every(point =>
        Array.isArray(point) && point.length === 3 && point.every(Number.isFinite));
      if (!validPath) return false;
      path = saved.path.map(normalize);
      closed = Boolean(saved.closed) && path.length >= 8;
      if (saved.values && typeof saved.values === 'object') {
        for (const id of DRAFT_FIELDS) {
          if (typeof saved.values[id] === 'string') fields[id].value = saved.values[id];
        }
      }
      fields.smoothValue.textContent = fields.smoothing.value;
      mechanismLengthWasEdited = Boolean(saved.values?.finalLegLength);
      return path.length > 0;
    } catch (_) {
      return false;
    }
  }

  function resize() {
    const rect = viewport.getBoundingClientRect();
    const scale = window.devicePixelRatio || 1;
    canvas.width = Math.round(rect.width * scale);
    canvas.height = Math.round(rect.height * scale);
    canvas.style.width = `${rect.width}px`;
    canvas.style.height = `${rect.height}px`;
    ctx.setTransform(scale, 0, 0, scale, 0, 0);
    geometry = {width: rect.width, height: rect.height, cx: rect.width / 2, cy: rect.height / 2, radius: Math.max(120, Math.min(rect.width, rect.height) * .38)};
    render();
  }

  function drawGrid(front) {
    ctx.save();
    ctx.strokeStyle = front ? 'rgba(64,101,82,.30)' : 'rgba(72,91,80,.12)';
    ctx.lineWidth = front ? 1 : .8;
    ctx.setLineDash(front ? [] : [3, 5]);
    const families = [];
    for (let latitude = -60; latitude <= 60; latitude += 30) {
      const ring = [];
      const phi = latitude * Math.PI / 180;
      for (let i = 0; i <= 96; i++) {
        const a = i * Math.PI * 2 / 96;
        ring.push([Math.cos(phi) * Math.cos(a), Math.sin(phi), Math.cos(phi) * Math.sin(a)]);
      }
      families.push(ring);
    }
    for (let longitude = 0; longitude < 180; longitude += 30) {
      const ring = [];
      const a = longitude * Math.PI / 180;
      for (let i = 0; i <= 96; i++) {
        const phi = -Math.PI / 2 + i * Math.PI / 96;
        ring.push([Math.cos(phi) * Math.cos(a), Math.sin(phi), Math.cos(phi) * Math.sin(a)]);
      }
      for (let i = 95; i >= 0; i--) {
        const phi = -Math.PI / 2 + i * Math.PI / 96;
        ring.push([-Math.cos(phi) * Math.cos(a), Math.sin(phi), -Math.cos(phi) * Math.sin(a)]);
      }
      families.push(ring);
    }
    for (const ring of families) drawSegments(ring, front, false);
    ctx.restore();
  }

  function drawSegments(points, front, closePath) {
    const count = closePath ? points.length : points.length - 1;
    for (let i = 0; i < count; i++) {
      const a = project(points[i]);
      const b = project(points[(i + 1) % points.length]);
      if (((a[2] + b[2]) >= 0) !== front) continue;
      ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke();
    }
  }

  function drawSphereScene() {
    const gradient = ctx.createRadialGradient(
      geometry.cx - geometry.radius * .32, geometry.cy - geometry.radius * .34, geometry.radius * .05,
      geometry.cx, geometry.cy, geometry.radius);
    gradient.addColorStop(0, '#edf8f2'); gradient.addColorStop(.7, '#c4ddd2'); gradient.addColorStop(1, '#91b8a7');
    ctx.beginPath(); ctx.arc(geometry.cx, geometry.cy, geometry.radius, 0, Math.PI * 2);
    ctx.fillStyle = gradient; ctx.fill();
    ctx.strokeStyle = '#6c9482'; ctx.lineWidth = 1.4; ctx.stroke();
    drawGrid(false); drawGrid(true);
    const outward = project([-1, 0, 0]);
    if (outward[2] >= 0) {
      ctx.beginPath(); ctx.arc(outward[0], outward[1], 5, 0, Math.PI * 2);
      ctx.fillStyle = '#c97838'; ctx.fill(); ctx.strokeStyle = '#fff'; ctx.lineWidth = 2; ctx.stroke();
      ctx.fillStyle = '#8a542d'; ctx.font = '700 10px Inter, sans-serif';
      ctx.fillText('Outward neutral direction', outward[0] + 10, outward[1] - 8);
    }
    const visiblePath = previewPath();
    if (visiblePath.length > 1) {
      ctx.lineCap = 'round'; ctx.lineJoin = 'round';
      ctx.strokeStyle = 'rgba(111,57,157,.42)'; ctx.lineWidth = 4; ctx.setLineDash([7, 7]);
      drawSegments(visiblePath, false, closed);
      ctx.strokeStyle = '#8e3ec0'; ctx.lineWidth = 4.5; ctx.setLineDash([]);
      drawSegments(visiblePath, true, closed);
    }
    if (visiblePath.length) {
      const first = project(visiblePath[0]);
      ctx.beginPath(); ctx.arc(first[0], first[1], 6, 0, Math.PI * 2);
      ctx.fillStyle = '#fff'; ctx.fill(); ctx.strokeStyle = '#71329b'; ctx.lineWidth = 3; ctx.stroke();
    }
  }

  const add3 = (a, b) => a.map((value, index) => value + b[index]);
  const scale3 = (value, scale) => value.map(component => component * scale);
  const rotateZ3 = (point, angle) => {
    const cosine = Math.cos(angle), sine = Math.sin(angle);
    return [cosine * point[0] - sine * point[1], sine * point[0] + cosine * point[1], point[2]];
  };

  function mechanismGeometry(settings) {
    const unitPath = previewPath();
    const count = unitPath.length;
    const pitchCurve = unitPath.map((unit, index) => {
      const joint = add3(settings.pivot, scale3(unit, -settings.armLength));
      return rotateZ3(joint, -2 * Math.PI * index / count);
    });
    const channelRadius = settings.ballRadius + settings.clearance;
    const pitchRadii = pitchCurve.map(point => Math.hypot(point[0], point[1]));
    const pitchZ = pitchCurve.map(point => point[2]);
    const minimumPitchRadius = Math.min(...pitchRadii);
    const maximumPitchRadius = Math.max(...pitchRadii);
    const minimumZ = Math.min(...pitchZ);
    const maximumZ = Math.max(...pitchZ);
    return {
      unitPath, pitchCurve, channelRadius, minimumPitchRadius, maximumPitchRadius,
      bottomZ: minimumZ - channelRadius - settings.edgeMargin,
      topZ: maximumZ + channelRadius + settings.edgeMargin,
      outerRadius: maximumPitchRadius + channelRadius + settings.edgeMargin
    };
  }

  function curveEnvelope(radius) {
    const points = previewPath();
    if (!points.length) return [0, 0, 0];
    return [0, 1, 2].map(axis => {
      const values = points.map(point => point[axis]);
      return (Math.max(...values) - Math.min(...values)) * radius;
    });
  }

  function bounds3(points) {
    const minimum = [Infinity, Infinity, Infinity];
    const maximum = [-Infinity, -Infinity, -Infinity];
    for (const point of points) {
      for (let axis = 0; axis < 3; axis++) {
        minimum[axis] = Math.min(minimum[axis], point[axis]);
        maximum[axis] = Math.max(maximum[axis], point[axis]);
      }
    }
    return {minimum, maximum, size: maximum.map((value, axis) => value - minimum[axis])};
  }

  function minimumPitchRadiusFor(pivotDistance, armLength) {
    return Math.min(...previewPath().map(unit => Math.hypot(
      -pivotDistance - armLength * unit[0],
      -armLength * unit[1]
    )));
  }

  function safeMechanismLimits(settings, requiredPitchRadius) {
    let low = settings.armLength + 0.1;
    let high = Math.max(low + requiredPitchRadius + 1, settings.pivotDistance, settings.armLength * 2);
    while (minimumPitchRadiusFor(high, settings.armLength) <= requiredPitchRadius && high < 100000) high *= 1.5;
    for (let pass = 0; pass < 50; pass++) {
      const middle = (low + high) / 2;
      if (minimumPitchRadiusFor(middle, settings.armLength) > requiredPitchRadius) high = middle;
      else low = middle;
    }
    const minimumPivotDistance = high;

    let maximumArmLength = 0;
    if (settings.pivotDistance > requiredPitchRadius) {
      const ceiling = Math.max(0.01, settings.pivotDistance - 0.1);
      const steps = 240;
      let safeArm = 0;
      let unsafeArm = ceiling;
      let foundUnsafe = false;
      for (let index = 1; index <= steps; index++) {
        const candidate = ceiling * index / steps;
        if (minimumPitchRadiusFor(settings.pivotDistance, candidate) <= requiredPitchRadius) {
          unsafeArm = candidate; foundUnsafe = true; break;
        }
        safeArm = candidate;
      }
      if (!foundUnsafe) maximumArmLength = ceiling;
      else {
        for (let pass = 0; pass < 40; pass++) {
          const middle = (safeArm + unsafeArm) / 2;
          if (minimumPitchRadiusFor(settings.pivotDistance, middle) > requiredPitchRadius) safeArm = middle;
          else unsafeArm = middle;
        }
        maximumArmLength = safeArm;
      }
    }
    return {minimumPivotDistance, maximumArmLength};
  }

  function analyzeMechanism(settings) {
    const geometryResult = mechanismGeometry(settings);
    const requiredPitchRadius = geometryResult.channelRadius + 1.0;
    const limits = safeMechanismLimits(settings, requiredPitchRadius);
    const failures = [];
    const maximumTiltDeg = Math.max(...geometryResult.unitPath.map(unit =>
      Math.acos(clamp(-unit[0], -1, 1)) * 180 / Math.PI));
    if (settings.radius <= 0) failures.push('Leg length must be positive.');
    if (settings.pivotDistance <= 0) failures.push('Cam-centre-to-pivot spacing must be positive.');
    if (settings.armLength <= 0) failures.push('Contact-ball-to-pivot length must be positive.');
    if (settings.ballRadius <= 0) failures.push('Contact-ball diameter must be positive.');
    if (settings.clearance < 0) failures.push('Groove clearance cannot be negative.');
    if (settings.edgeMargin <= 0) failures.push('Outer edge thickness must be positive.');
    if (settings.armLength <= settings.ballRadius + 0.5) failures.push('Follower arm is too short for the contact ball.');
    if (settings.pivotDistance <= settings.armLength) failures.push('The leg pivot must remain farther from the shaft than the contact ball.');
    if (geometryResult.unitPath.some(unit => unit[0] >= -0.02)) failures.push('Part of the foot path crosses behind the leg pivot. Redraw it on the outward side of the sphere.');
    if (geometryResult.minimumPitchRadius <= requiredPitchRadius) failures.push('The groove passes too close to the cam shaft.');
    return {
      ...geometryResult,
      ...limits,
      requiredPitchRadius,
      failures,
      safe: failures.length === 0,
      camDiameter: geometryResult.outerRadius * 2,
      camWidth: geometryResult.topZ - geometryResult.bottomZ,
      footEnvelope: curveEnvelope(settings.radius),
      maximumTiltDeg
    };
  }

  function drawMechanismScene() {
    if (!closed || !appliedMechanism) return;
    const settings = appliedMechanism;
    const mechanism = mechanismGeometry(settings);
    const count = mechanism.unitPath.length;
    const sampleIndex = Math.floor(animationPhase * count) % count;
    const unit = mechanism.unitPath[sampleIndex];
    const footPath = mechanism.unitPath.map(point => add3(settings.pivot, scale3(point, settings.radius)));
    const gaitBounds = bounds3(footPath);
    const gaitCorners = [];
    for (let bits = 0; bits < 8; bits++) {
      gaitCorners.push([0, 1, 2].map(axis =>
        bits & (1 << axis) ? gaitBounds.maximum[axis] : gaitBounds.minimum[axis]));
    }
    const gaitPadding = Math.max(2, Math.max(...gaitBounds.size) * .08, settings.ballRadius * 1.5);
    const dimensionLines = [
      {
        start: [gaitBounds.minimum[0], gaitBounds.minimum[1] - gaitPadding, gaitBounds.minimum[2]],
        end: [gaitBounds.maximum[0], gaitBounds.minimum[1] - gaitPadding, gaitBounds.minimum[2]],
        label: `X ${gaitBounds.size[0].toFixed(1)} mm`, color: '#c4544d'
      },
      {
        start: [gaitBounds.minimum[0] - gaitPadding, gaitBounds.minimum[1], gaitBounds.minimum[2]],
        end: [gaitBounds.minimum[0] - gaitPadding, gaitBounds.maximum[1], gaitBounds.minimum[2]],
        label: `Y ${gaitBounds.size[1].toFixed(1)} mm`, color: '#39805f'
      },
      {
        start: [gaitBounds.maximum[0] + gaitPadding, gaitBounds.maximum[1] + gaitPadding, gaitBounds.minimum[2]],
        end: [gaitBounds.maximum[0] + gaitPadding, gaitBounds.maximum[1] + gaitPadding, gaitBounds.maximum[2]],
        label: `Z ${gaitBounds.size[2].toFixed(1)} mm`, color: '#397ab5'
      }
    ];
    const foot = footPath[sampleIndex];
    const joint = add3(settings.pivot, scale3(unit, -settings.armLength));
    const camRotation = 2 * Math.PI * animationPhase;
    const rotatingPitch = mechanism.pitchCurve.map(point => rotateZ3(point, camRotation));
    const camCircleBottom = [], camCircleTop = [];
    for (let index = 0; index <= 96; index++) {
      const angle = 2 * Math.PI * index / 96 + camRotation;
      camCircleBottom.push([mechanism.outerRadius * Math.cos(angle), mechanism.outerRadius * Math.sin(angle), mechanism.bottomZ]);
      camCircleTop.push([mechanism.outerRadius * Math.cos(angle), mechanism.outerRadius * Math.sin(angle), mechanism.topZ]);
    }
    const scenePoints = [
      ...footPath, ...gaitCorners, ...dimensionLines.flatMap(line => [line.start, line.end]),
      ...rotatingPitch, ...camCircleBottom, ...camCircleTop, settings.pivot, foot, joint,
      [0,0,mechanism.bottomZ - 12], [0,0,mechanism.topZ + 12]
    ];
    const cameraPoints = scenePoints.map(worldToCamera);
    const xs = cameraPoints.map(point => point[0]), ys = cameraPoints.map(point => point[1]);
    const minX = Math.min(...xs), maxX = Math.max(...xs), minY = Math.min(...ys), maxY = Math.max(...ys);
    const spanX = Math.max(1, maxX - minX), spanY = Math.max(1, maxY - minY);
    const scale = Math.min((geometry.width - 100) / spanX, (geometry.height - 100) / spanY);
    const sceneCx = geometry.width / 2 - (minX + maxX) * scale / 2;
    const sceneCy = geometry.height / 2 + (minY + maxY) * scale / 2;
    const worldProject = point => {
      const camera = worldToCamera(point);
      return [sceneCx + camera[0] * scale, sceneCy - camera[1] * scale, camera[2]];
    };
    const strokePath = (points, color, width, close = false, dash = []) => {
      if (!points.length) return;
      ctx.beginPath();
      const first = worldProject(points[0]); ctx.moveTo(first[0], first[1]);
      for (let index = 1; index < points.length; index++) {
        const projected = worldProject(points[index]); ctx.lineTo(projected[0], projected[1]);
      }
      if (close) ctx.closePath();
      ctx.strokeStyle = color; ctx.lineWidth = width; ctx.setLineDash(dash); ctx.stroke(); ctx.setLineDash([]);
    };
    const fillPath = (points, color) => {
      if (!points.length) return;
      ctx.beginPath();
      const first = worldProject(points[0]); ctx.moveTo(first[0], first[1]);
      for (let index = 1; index < points.length; index++) {
        const projected = worldProject(points[index]); ctx.lineTo(projected[0], projected[1]);
      }
      ctx.closePath(); ctx.fillStyle = color; ctx.fill();
    };
    const drawMarker = (point, radiusMm, fill, outline = '#fff') => {
      const projected = worldProject(point);
      ctx.beginPath(); ctx.arc(projected[0], projected[1], Math.max(5, radiusMm * scale), 0, Math.PI * 2);
      ctx.fillStyle = fill; ctx.fill(); ctx.strokeStyle = outline; ctx.lineWidth = 2; ctx.stroke();
      return projected;
    };
    const drawLabel = (text, point, offsetX, offsetY, color = '#24332b') => {
      const projected = worldProject(point);
      ctx.fillStyle = color; ctx.font = '700 11px Inter, sans-serif';
      ctx.fillText(text, projected[0] + offsetX, projected[1] + offsetY);
    };
    const drawDimension = ({start, end, label, color}) => {
      const a = worldProject(start), b = worldProject(end);
      const dx = b[0] - a[0], dy = b[1] - a[1];
      const length = Math.hypot(dx, dy);
      // An axis can point directly into the screen in face view. Its value is
      // still always visible in the gait-size badge.
      if (length < 28) return;
      const ux = dx / length, uy = dy / length;
      ctx.save();
      ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = 1.4; ctx.setLineDash([]);
      ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke();
      const arrow = (point, direction) => {
        const px = -uy, py = ux;
        ctx.beginPath();
        ctx.moveTo(point[0], point[1]);
        ctx.lineTo(point[0] + direction * ux * 8 + px * 4, point[1] + direction * uy * 8 + py * 4);
        ctx.lineTo(point[0] + direction * ux * 8 - px * 4, point[1] + direction * uy * 8 - py * 4);
        ctx.closePath(); ctx.fill();
      };
      arrow(a, 1); arrow(b, -1);
      const mx = (a[0] + b[0]) / 2, my = (a[1] + b[1]) / 2;
      ctx.font = '700 11px Inter, sans-serif';
      const width = ctx.measureText(label).width;
      ctx.globalAlpha = .92; ctx.fillStyle = '#fff';
      ctx.fillRect(mx - width / 2 - 5, my - 9, width + 10, 18);
      ctx.globalAlpha = 1; ctx.fillStyle = color; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      ctx.fillText(label, mx, my); ctx.restore();
    };

    const gradient = ctx.createLinearGradient(0, 0, geometry.width, geometry.height);
    gradient.addColorStop(0, '#f9fbf9'); gradient.addColorStop(1, '#e8eeea');
    ctx.fillStyle = gradient; ctx.fillRect(0, 0, geometry.width, geometry.height);

    ctx.save(); ctx.globalAlpha = .12;
    fillPath(camCircleBottom, '#72aa9c');
    fillPath(camCircleTop, '#72aa9c');
    ctx.globalAlpha = .32;
    strokePath(camCircleBottom, '#4e8b7d', 2, true);
    strokePath(camCircleTop, '#4e8b7d', 2, true);
    for (let index = 0; index < 96; index += 12) strokePath([camCircleBottom[index], camCircleTop[index]], '#4e8b7d', 1);
    ctx.restore();
    // Axis-aligned physical envelope of the complete foot motion.
    ctx.save(); ctx.strokeStyle = 'rgba(72,93,82,.62)'; ctx.lineWidth = 1.2; ctx.setLineDash([5, 4]);
    for (let corner = 0; corner < 8; corner++) {
      for (let axis = 0; axis < 3; axis++) {
        if ((corner & (1 << axis)) === 0) strokePath(
          [gaitCorners[corner], gaitCorners[corner | (1 << axis)]],
          'rgba(72,93,82,.62)', 1.2, false, [5, 4]);
      }
    }
    ctx.restore();
    dimensionLines.forEach(drawDimension);
    strokePath(rotatingPitch, '#73379b', Math.max(3, mechanism.channelRadius * scale * 1.3), true);
    strokePath(footPath, '#a23bbd', 4, true, [7, 6]);
    strokePath([[0,0,0], settings.pivot], '#7c8b83', 1.5, false, [5, 5]);
    // Contact ball, pivot, and foot are one straight follower leg.
    strokePath([joint, foot], '#c97838', Math.max(5, settings.ballRadius * scale * .7));
    strokePath([[0,0,mechanism.bottomZ - 12], [0,0,mechanism.topZ + 12]], '#35453d', 8);

    drawMarker(settings.pivot, Math.max(2, settings.ballRadius * .75), '#e5a15f');
    drawMarker(joint, settings.ballRadius, '#397ab5');
    drawMarker(foot, Math.max(2, settings.ballRadius * .65), '#a23bbd');
    drawMarker([0,0,0], Math.max(2, settings.ballRadius * .7), '#24332b');
    drawLabel('Leg pivot', settings.pivot, -55, -14);
    drawLabel('Contact ball', joint, 10, 2, '#24577f');
    drawLabel('Foot', foot, 10, -10, '#77258e');
    drawLabel('Cam shaft', [0,0,0], 10, 18);
  }

  function render() {
    ctx.clearRect(0, 0, geometry.width, geometry.height);
    if (stage === 'draw') drawSphereScene();
    else drawMechanismScene();
    updateStatus();
  }

  function animationTick(time) {
    if (!animationPlaying) return;
    if (previousAnimationTime) animationPhase = (animationPhase + (time - previousAnimationTime) / 6000) % 1;
    previousAnimationTime = time;
    fields.phaseSlider.value = Math.round(animationPhase * 359);
    fields.phaseValue.textContent = `${Math.round(animationPhase * 360)}°`;
    render();
    requestAnimationFrame(animationTick);
  }

  function setAnimationPlaying(playing) {
    animationPlaying = playing;
    buttons.playButton.textContent = playing ? '❚❚ Pause' : '▶ Play';
    buttons.playButton.setAttribute('aria-pressed', playing);
    previousAnimationTime = 0;
    if (playing) requestAnimationFrame(animationTick);
  }

  buttons.playButton.onclick = () => setAnimationPlaying(!animationPlaying);
  buttons.faceViewButton.onclick = () => {
    yaw = 0; pitch = 0; render();
  };
  fields.phaseSlider.oninput = () => {
    setAnimationPlaying(false);
    animationPhase = Number(fields.phaseSlider.value) / 359;
    fields.phaseValue.textContent = `${Math.round(animationPhase * 360)}°`;
    render();
  };

  function spherePoint(event) {
    const rect = canvas.getBoundingClientRect();
    const x = (event.clientX - rect.left - geometry.cx) / geometry.radius;
    const y = -(event.clientY - rect.top - geometry.cy) / geometry.radius;
    const d2 = x * x + y * y;
    if (d2 > 1) return null;
    return cameraToWorld([x, y, Math.sqrt(Math.max(0, 1 - d2))]);
  }

  function addPoint(point) {
    if (!point || closed) return;
    if (!path.length || Math.acos(clamp(path[path.length - 1].reduce((s, v, i) => s + v * point[i], 0), -1, 1)) > .006) {
      path.push(point);
    }
  }

  canvas.addEventListener('pointerdown', event => {
    dragging = true; previousPointer = [event.clientX, event.clientY]; canvas.setPointerCapture(event.pointerId);
    if (mode === 'draw') addPoint(spherePoint(event));
    render();
  });
  canvas.addEventListener('pointermove', event => {
    if (!dragging) return;
    if (mode === 'draw') addPoint(spherePoint(event));
    else {
      yaw += (event.clientX - previousPointer[0]) * .008;
      pitch = clamp(pitch + (event.clientY - previousPointer[1]) * .008, -1.45, 1.45);
      previousPointer = [event.clientX, event.clientY];
    }
    render();
  });
  canvas.addEventListener('pointerup', () => {
    dragging = false; previousPointer = null; saveDraft();
  });

  function setMode(next) {
    mode = next;
    buttons.drawTool.classList.toggle('active', mode === 'draw');
    buttons.orbitTool.classList.toggle('active', mode === 'orbit');
    buttons.drawTool.setAttribute('aria-pressed', mode === 'draw');
    buttons.orbitTool.setAttribute('aria-pressed', mode === 'orbit');
    document.getElementById('modeHint').textContent = mode === 'draw' ? 'Draw around the outward neutral marker' : 'Drag to rotate the view';
  }
  buttons.drawTool.onclick = () => setMode('draw');
  buttons.orbitTool.onclick = () => setMode('orbit');
  buttons.undoButton.onclick = () => {
    if (closed) closed = false; else path.splice(Math.max(0, path.length - 12));
    saveDraft(); render();
  };
  buttons.closeButton.onclick = () => {
    if (path.length >= 8) {
      closed = true; fields.message.textContent = 'Loop closed. Export the curve or continue to the mechanism.';
      saveDraft(); render();
    }
  };
  buttons.clearButton.onclick = () => {
    path = []; closed = false; fields.message.textContent = 'Draw one continuous path, then close the loop.';
    saveDraft(); render();
  };

  function smoothUnitPath(input, iterations) {
    let result = input.map(p => [...p]);
    for (let pass = 0; pass < iterations; pass++) {
      result = result.map((p, i) => {
        const a = result[(i + result.length - 1) % result.length];
        const b = result[(i + 1) % result.length];
        return normalize([a[0] * .25 + p[0] * .5 + b[0] * .25, a[1] * .25 + p[1] * .5 + b[1] * .25, a[2] * .25 + p[2] * .5 + b[2] * .25]);
      });
    }
    return result;
  }

  function resampleClosed(input, count) {
    const lengths = [0];
    for (let i = 0; i < input.length; i++) {
      const a = input[i], b = input[(i + 1) % input.length];
      lengths.push(lengths[lengths.length - 1] + Math.acos(clamp(a.reduce((s,v,j) => s + v * b[j], 0), -1, 1)));
    }
    const total = lengths[lengths.length - 1];
    const output = [];
    let segment = 0;
    for (let n = 0; n < count; n++) {
      const target = total * n / count;
      while (segment + 1 < lengths.length && lengths[segment + 1] < target) segment++;
      const a = input[segment % input.length], b = input[(segment + 1) % input.length];
      const fraction = (target - lengths[segment]) / Math.max(1e-12, lengths[segment + 1] - lengths[segment]);
      const omega = Math.acos(clamp(a.reduce((s,v,j) => s + v * b[j], 0), -1, 1));
      if (omega < 1e-7) output.push([...a]);
      else {
        const so = Math.sin(omega);
        output.push(normalize(a.map((v,j) => v * Math.sin((1-fraction)*omega)/so + b[j] * Math.sin(fraction*omega)/so)));
      }
    }
    return output;
  }

  function previewPath() {
    if (!closed || path.length < 8) return path;
    const smoothed = smoothUnitPath(path, Math.round(numeric(fields.smoothing, 2)));
    const count = clamp(Math.round(numeric(fields.sampleCount, 360)), 36, 1440);
    return resampleClosed(smoothed, count);
  }

  function scaledPoints(radius, center = [0, 0, 0]) {
    return previewPath().map(p => p.map((v,i) => center[i] + radius * v));
  }
  function curvePoints() {
    return scaledPoints(Math.max(1, numeric(fields.sphereRadius, 150)));
  }
  function mechanismSettings() {
    const radius = numeric(fields.finalLegLength, 150);
    const pivotDistance = numeric(fields.pivotDistance, 80);
    const armLength = numeric(fields.armLength, 40);
    const pivot = [-pivotDistance, 0, 0];
    const joint = [-pivotDistance + armLength, 0, 0];
    return {
      radius, pivotDistance, armLength, pivot, joint,
      ballRadius: numeric(fields.ballDiameter, 5) / 2,
      clearance: numeric(fields.printClearance, 0.25),
      edgeMargin: numeric(fields.edgeMargin, 2.3)
    };
  }
  function csvText(points) {
    return 'index,x_mm,y_mm,z_mm\n' + points.map((p,i) => `${i},${p[0].toFixed(9)},${p[1].toFixed(9)},${p[2].toFixed(9)}`).join('\n') + '\n';
  }
  function txtText(points) {
    return '# CamStudio spherical foot path\n# x_mm y_mm z_mm\n' + points.map(p => p.map(value => value.toFixed(9)).join(' ')).join('\n') + '\n';
  }
  function downloadText(text, filename, type) {
    const blob = new Blob([text], {type});
    const link = document.createElement('a');
    link.href = URL.createObjectURL(blob); link.download = filename; link.click();
    URL.revokeObjectURL(link.href);
  }
  buttons.downloadButton.onclick = () => {
    downloadText(csvText(curvePoints()), `${safeName()}.csv`, 'text/csv');
  };
  buttons.downloadTxtButton.onclick = () => downloadText(txtText(curvePoints()), `${safeName()}.txt`, 'text/plain');
  const safeName = () => (fields.designName.value || 'sphere-curve').trim().replace(/[^a-zA-Z0-9_-]+/g, '-').replace(/^-+|-+$/g, '') || 'sphere-curve';

  function setStage(next) {
    stage = next;
    if (stage === 'mechanism' && !mechanismLengthWasEdited) {
      fields.finalLegLength.value = Math.max(1, numeric(fields.sphereRadius, 150));
    }
    fields.drawPanel.hidden = stage !== 'draw';
    fields.mechanismPanel.hidden = stage !== 'mechanism';
    fields.drawActions.hidden = stage !== 'draw';
    fields.mechanismActions.hidden = stage !== 'mechanism';
    fields.stepOneBadge.classList.toggle('active', stage === 'draw');
    fields.stepTwoBadge.classList.toggle('active', stage === 'mechanism');
    fields.previewLegend.hidden = stage !== 'mechanism';
    fields.gaitEnvelopeBadge.hidden = stage !== 'mechanism';
    buttons.drawTool.disabled = stage === 'mechanism';
    buttons.undoButton.disabled = stage === 'mechanism';
    buttons.closeButton.disabled = stage === 'mechanism';
    buttons.clearButton.disabled = stage === 'mechanism';
    if (stage === 'mechanism') {
      yaw = 0; pitch = 0;
      setMode('orbit');
      applyMechanism(true);
    } else {
      setAnimationPlaying(false);
      yaw = Math.PI / 2; pitch = 0;
      setMode('draw');
    }
    fields.message.textContent = stage === 'draw'
      ? (closed ? 'Curve ready. Export it or continue to the mechanism.' : 'Draw one continuous path, then close the loop.')
      : (serverReady ? 'Set the mechanism dimensions, then generate the STEP file.' : 'STEP generation is available in the launched CamStudio app.');
    saveDraft();
    render();
  }
  buttons.continueButton.onclick = () => setStage('mechanism');
  buttons.backButton.onclick = () => setStage('draw');

  function applyMechanism(silent = false) {
    if (!closed) return;
    appliedMechanism = mechanismSettings();
    appliedAnalysis = analyzeMechanism(appliedMechanism);
    mechanismDirty = false;
    fields.applyState.textContent = appliedAnalysis.safe
      ? 'Applied. The preview and animation use these dimensions.'
      : 'Applied, but the dimensions need correction before STEP generation.';
    fields.applyState.classList.toggle('dirty', !appliedAnalysis.safe);
    if (!silent) fields.message.textContent = appliedAnalysis.safe
      ? 'Preview updated. Press Play to inspect the complete motion.'
      : appliedAnalysis.failures[0];
    animationPhase = 0;
    fields.phaseSlider.value = 0;
    fields.phaseValue.textContent = '0°';
    updateMechanismGuardrails();
    render();
  }
  buttons.applyButton.onclick = () => applyMechanism(false);

  buttons.generateButton.onclick = async () => {
    buttons.generateButton.disabled = true; buttons.generateButton.classList.add('generating');
    buttons.revealButton.hidden = true;
    fields.message.textContent = 'Generating motion, groove, and STEP…';
    try {
      const settings = appliedMechanism;
      if (!settings || mechanismDirty || !appliedAnalysis?.safe) throw new Error('Apply valid mechanism dimensions before generating STEP');
      const response = await fetch('/api/generate', {
        method: 'POST', headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({
          name: safeName(), points: scaledPoints(settings.radius, settings.pivot),
          pivot: settings.pivot, joint: settings.joint,
          ballRadius: settings.ballRadius, clearance: settings.clearance, edgeMargin: settings.edgeMargin
        })
      });
      const result = await response.json();
      if (!response.ok) throw new Error(result.error || 'Generation failed');
      lastGeneratedName = safeName();
      buttons.revealButton.hidden = false;
      fields.message.textContent = `STEP ready: ${result.step}`;
    } catch (error) {
      fields.message.textContent = error.message + (location.protocol === 'file:' ? ' — open “Launch CamStudio.command” to generate STEP.' : '');
    } finally {
      buttons.generateButton.classList.remove('generating'); updateStatus();
    }
  };

  buttons.revealButton.onclick = async () => {
    if (!lastGeneratedName) return;
    try {
      const response = await fetch('/api/reveal', {
        method: 'POST', headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({name: lastGeneratedName})
      });
      const result = await response.json();
      if (!response.ok) throw new Error(result.error || 'Could not open the output folder');
      fields.message.textContent = 'The generated STEP is selected in Finder.';
    } catch (error) {
      fields.message.textContent = error.message;
    }
  };

  function formatEnvelope(envelope) {
    return envelope.map(value => Number(value.toFixed(1))).join(' × ') + ' mm';
  }

  function updateMechanismGuardrails() {
    if (!appliedMechanism || !appliedAnalysis) return;
    const sketchRadius = Math.max(1, numeric(fields.sphereRadius, 150));
    const scalePercent = appliedMechanism.radius / sketchRadius * 100;
    fields.designState.textContent = appliedAnalysis.safe ? 'Mechanism dimensions safe' : 'Change dimensions';
    fields.designStateDot.className = `check-dot ${appliedAnalysis.safe ? 'safe' : 'error'}`;
    fields.legRule.textContent = `Leg length has no single mathematical value: any positive length makes the same angular motion. ${Number(appliedMechanism.radius.toFixed(1))} mm produces ${Number(scalePercent.toFixed(0))}% of the Step 1 path size (${formatEnvelope(appliedAnalysis.footEnvelope)}).`;
    const clearanceMargin = appliedAnalysis.minimumPitchRadius - appliedAnalysis.requiredPitchRadius;
    fields.orientationRule.textContent = `Contact ball → leg pivot → foot is one straight line. This curve tilts that leg up to ${Number(appliedAnalysis.maximumTiltDeg.toFixed(1))}° away from its outward neutral direction.`;
    fields.pivotRule.textContent = `For this curve: cam-centre-to-pivot spacing must be at least ${Number(appliedAnalysis.minimumPivotDistance.toFixed(1))} mm with the current contact segment. At ${Number(appliedMechanism.pivotDistance.toFixed(1))} mm spacing, that segment can be up to ${Number(appliedAnalysis.maximumArmLength.toFixed(1))} mm. Current groove safety margin: ${Number(clearanceMargin.toFixed(2))} mm.`;
    fields.camEstimate.textContent = appliedAnalysis.safe
      ? `Estimated cam: ${Number(appliedAnalysis.camDiameter.toFixed(1))} mm diameter × ${Number(appliedAnalysis.camWidth.toFixed(1))} mm wide.`
      : appliedAnalysis.failures.join(' ');
  }

  function updateStatus() {
    const outputCount = closed ? previewPath().length : path.length;
    fields.pointCount.textContent = closed ? `${outputCount} output` : `${outputCount} drawn`;
    fields.motionEnvelope.textContent = path.length > 1 ? formatEnvelope(curveEnvelope(Math.max(1, numeric(fields.sphereRadius, 150)))) : '—';
    fields.sphereError.textContent = '0.000 mm';
    fields.loopStatus.textContent = closed ? 'Closed' : 'Open';
    fields.loopStatus.className = closed ? 'ready' : 'waiting';
    const valid = closed && path.length >= 8;
    buttons.downloadButton.disabled = !valid;
    buttons.downloadTxtButton.disabled = !valid;
    buttons.continueButton.disabled = !valid;
    buttons.generateButton.disabled = !valid || !serverReady || mechanismDirty || !appliedAnalysis?.safe;
    buttons.closeButton.disabled = stage === 'mechanism' || path.length < 8 || closed;
    if (stage === 'draw') {
      const radius = numeric(fields.sphereRadius, 150);
      fields.liveScale.textContent = `Estimated movement radius ${Number(radius.toFixed(2))} mm`;
    } else {
      const settings = appliedMechanism || mechanismSettings();
      fields.liveScale.textContent = `Leg ${Number(settings.radius.toFixed(2))} mm · Pivot X −${Number(settings.pivotDistance.toFixed(2))} mm`;
      fields.mechanismSummary.textContent = `Pivot X −${Number(settings.pivotDistance.toFixed(2))} mm · Contact ball X ${settings.joint[0] < 0 ? '−' : ''}${Math.abs(Number(settings.joint[0].toFixed(2)))} mm`;
      updateMechanismGuardrails();
      fields.gaitEnvelopeBadge.textContent = `Gait box · X ${appliedAnalysis.footEnvelope[0].toFixed(1)} × Y ${appliedAnalysis.footEnvelope[1].toFixed(1)} × Z ${appliedAnalysis.footEnvelope[2].toFixed(1)} mm`;
    }
  }
  fields.smoothing.oninput = () => {
    fields.smoothValue.textContent = fields.smoothing.value; saveDraft(); render();
  };
  fields.finalLegLength.addEventListener('input', () => { mechanismLengthWasEdited = true; });
  ['sampleCount','sphereRadius'].forEach(id => {
    fields[id].addEventListener('input', () => { saveDraft(); render(); });
    fields[id].addEventListener('change', () => { saveDraft(); render(); });
  });
  ['finalLegLength','pivotDistance','armLength','ballDiameter','printClearance','edgeMargin'].forEach(id => {
    const markDirty = () => {
      mechanismDirty = true;
      fields.applyState.textContent = 'Values changed — press Apply to update the preview.';
      fields.applyState.classList.add('dirty');
      saveDraft();
      updateStatus();
    };
    fields[id].addEventListener('input', markDirty);
    fields[id].addEventListener('change', markDirty);
  });
  fields.designName.addEventListener('input', saveDraft);

  async function detectServer() {
    if (location.protocol === 'file:') {
      fields.runtimeBanner.hidden = false;
      fields.runtimeBanner.textContent = 'Preview mode only — open “Launch CamStudio.command” for STEP generation.';
      updateStatus();
      return;
    }
    try {
      const response = await fetch('/api/status', {cache: 'no-store'});
      const result = await response.json();
      serverReady = response.ok && result.ready === true;
    } catch (_) {
      serverReady = false;
    }
    fields.runtimeBanner.hidden = serverReady;
    if (!serverReady) fields.runtimeBanner.textContent = 'Cam generator is not connected. Start CamStudio with the launcher.';
    updateStatus();
  }
  window.addEventListener('resize', resize);
  new ResizeObserver(resize).observe(viewport);
  const recoveredDraft = restoreDraft();
  if (recoveredDraft) fields.message.textContent = 'Recovered your saved sketch. Continue editing or use the closed curve.';
  resize();
  detectServer();
})();
