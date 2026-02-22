import init, { GridWorld } from "./pkg/demo_wasm.js";

const GRID_W = 40;
const GRID_H = 25;

const COLORS = {
  floor: "#f3f1eb",
  wall: "#1f2937",
  start: "#f59e0b",
  goal: "#ef4444",
  astar: "#2563eb",
  jps: "#16a34a",
  both: "#0ea5e9",
  origin: "#a855f7",
  invisible: "rgba(15, 23, 42, 0.42)",
};

await init();

const world = new GridWorld(GRID_W, GRID_H);
let activeTab = "path";

const tabs = [...document.querySelectorAll(".tab")];
const panels = {
  path: document.getElementById("panel-path"),
  fov: document.getElementById("panel-fov"),
  gen: document.getElementById("panel-gen"),
  hex: document.getElementById("panel-hex"),
};

const squareCanvas = document.getElementById("squareCanvas");
const fovCanvas = document.getElementById("fovCanvas");
const genCanvas = document.getElementById("genCanvas");
const hexCanvas = document.getElementById("hexCanvas");

const pathStats = document.getElementById("pathStats");
const genStats = document.getElementById("genStats");
const hexStats = document.getElementById("hexStats");

const fovRadiusInput = document.getElementById("fovRadius");
const genAlgo = document.getElementById("genAlgo");
const hexRadiusInput = document.getElementById("hexRadius");
const goalQInput = document.getElementById("goalQ");
const goalRInput = document.getElementById("goalR");

tabs.forEach((tabBtn) => {
  tabBtn.addEventListener("click", () => {
    const tab = tabBtn.dataset.tab;
    activeTab = tab;
    tabs.forEach((t) => t.classList.toggle("active", t === tabBtn));
    Object.entries(panels).forEach(([name, panel]) => {
      panel.classList.toggle("active", name === tab);
    });
    renderActive();
  });
});

document.getElementById("clearWalls").addEventListener("click", () => {
  world.clear_walls();
  renderSquare(squareCanvas, "path");
});

document.getElementById("randomWalls").addEventListener("click", () => {
  world.clear_walls();
  for (let y = 0; y < GRID_H; y += 1) {
    for (let x = 0; x < GRID_W; x += 1) {
      if (Math.random() < 0.2) {
        world.toggle_wall(x, y);
      }
    }
  }
  renderSquare(squareCanvas, "path");
});

fovRadiusInput.addEventListener("input", () => renderSquare(fovCanvas, "fov"));

document.getElementById("regenerate").addEventListener("click", () => {
  const seed = Date.now() >>> 0;
  switch (genAlgo.value) {
    case "bsp":
      world.generate_bsp(seed);
      break;
    case "cellular":
      world.generate_cellular(seed);
      break;
    case "drunkard":
      world.generate_drunkard(seed);
      break;
    default:
      break;
  }
  renderSquare(genCanvas, "gen");
});

document.getElementById("hexRecalc").addEventListener("click", drawHex);
hexRadiusInput.addEventListener("input", drawHex);

[squareCanvas, fovCanvas, genCanvas].forEach((canvas) => {
  canvas.addEventListener("click", (event) => {
    const coord = clickToCell(canvas, event, world.width(), world.height());
    if (!coord) {
      return;
    }
    const { x, y } = coord;

    if (activeTab === "path") {
      if (event.shiftKey) {
        world.set_start(x, y);
      } else if (event.ctrlKey || event.metaKey) {
        world.set_goal(x, y);
      } else {
        world.toggle_wall(x, y);
      }
      renderSquare(squareCanvas, "path");
      return;
    }

    if (activeTab === "fov") {
      if (event.shiftKey) {
        world.set_origin(x, y);
      } else {
        world.toggle_wall(x, y);
      }
      renderSquare(fovCanvas, "fov");
      return;
    }

    if (activeTab === "gen") {
      world.toggle_wall(x, y);
      renderSquare(genCanvas, "gen");
    }
  });
});

function renderActive() {
  if (activeTab === "path") {
    renderSquare(squareCanvas, "path");
  } else if (activeTab === "fov") {
    renderSquare(fovCanvas, "fov");
  } else if (activeTab === "gen") {
    renderSquare(genCanvas, "gen");
  } else {
    drawHex();
  }
}

function renderSquare(canvas, mode) {
  const ctx = setupCanvas2D(canvas);
  const w = world.width();
  const h = world.height();
  const render = world.render();
  const cellW = canvas.width / w;
  const cellH = canvas.height / h;

  let fovMask = null;
  if (mode === "fov") {
    fovMask = world.fov_mask(Number(fovRadiusInput.value));
  }

  for (let y = 0; y < h; y += 1) {
    for (let x = 0; x < w; x += 1) {
      const idx = y * w + x;
      const tile = render[idx];
      ctx.fillStyle = tileColor(tile);
      ctx.fillRect(x * cellW, y * cellH, cellW + 1, cellH + 1);

      if (fovMask && !fovMask[idx]) {
        ctx.fillStyle = COLORS.invisible;
        ctx.fillRect(x * cellW, y * cellH, cellW + 1, cellH + 1);
      }
    }
  }

  pathStats.textContent = `A*: cost ${world.astar_cost()} / nodes ${world.astar_nodes()} | JPS: cost ${world.jps_cost()} / nodes ${world.jps_nodes()}`;

  if (mode === "gen") {
    const floors = render.filter((v) => v !== 1).length;
    const ratio = ((floors / render.length) * 100).toFixed(1);
    genStats.textContent = `Floor ratio: ${ratio}%`;
  }
}

function drawHex() {
  const ctx = setupCanvas2D(hexCanvas);
  const radius = Number(hexRadiusInput.value);
  const goalQ = Number(goalQInput.value);
  const goalR = Number(goalRInput.value);

  ctx.clearRect(0, 0, hexCanvas.width, hexCanvas.height);

  const coordsFlat = world.hex_spiral(0, 0, radius);
  const pathFlat = world.hex_path(radius, 0, 0, goalQ, goalR, []);
  const coords = pairs(coordsFlat);
  const pathKeys = new Set(pairs(pathFlat).map(([q, r]) => `${q},${r}`));
  const goalKey = `${goalQ},${goalR}`;

  const size = Math.min(
    hexCanvas.width / Math.max(12, radius * 4),
    hexCanvas.height / Math.max(8, radius * 3.5),
  );
  const centerX = hexCanvas.width * 0.5;
  const centerY = hexCanvas.height * 0.5;

  coords.forEach(([q, r]) => {
    const { x, y } = axialToPixel(q, r, size, centerX, centerY);
    const key = `${q},${r}`;
    let fill = "#f7f5ef";
    if (key === "0,0") {
      fill = COLORS.start;
    } else if (key === goalKey) {
      fill = COLORS.goal;
    } else if (pathKeys.has(key)) {
      fill = COLORS.astar;
    }
    drawHexCell(ctx, x, y, size * 0.92, fill, "#4b5563");
  });

  const distance = world.hex_distance(0, 0, goalQ, goalR);
  const pathLen = pathFlat.length / 2;
  hexStats.textContent = `Hex distance: ${distance} | Path cells: ${pathLen}`;
}

function tileColor(tile) {
  switch (tile) {
    case 1:
      return COLORS.wall;
    case 2:
      return COLORS.start;
    case 3:
      return COLORS.goal;
    case 4:
      return COLORS.astar;
    case 5:
      return COLORS.jps;
    case 6:
      return COLORS.both;
    case 7:
      return COLORS.origin;
    default:
      return COLORS.floor;
  }
}

function clickToCell(canvas, event, width, height) {
  const rect = canvas.getBoundingClientRect();
  const x = Math.floor(((event.clientX - rect.left) / rect.width) * width);
  const y = Math.floor(((event.clientY - rect.top) / rect.height) * height);
  if (x < 0 || y < 0 || x >= width || y >= height) {
    return null;
  }
  return { x, y };
}

function setupCanvas2D(canvas) {
  const rect = canvas.getBoundingClientRect();
  const width = Math.max(1, Math.floor(rect.width));
  const height = Math.max(1, Math.floor(rect.height));
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  const ctx = canvas.getContext("2d");
  return ctx;
}

function pairs(flat) {
  const out = [];
  for (let i = 0; i + 1 < flat.length; i += 2) {
    out.push([flat[i], flat[i + 1]]);
  }
  return out;
}

function axialToPixel(q, r, size, cx, cy) {
  return {
    x: cx + size * (1.5 * q),
    y: cy + size * (Math.sqrt(3) * (r + q * 0.5)),
  };
}

function drawHexCell(ctx, x, y, size, fill, stroke) {
  ctx.beginPath();
  for (let i = 0; i < 6; i += 1) {
    const angle = (Math.PI / 180) * (60 * i);
    const px = x + size * Math.cos(angle);
    const py = y + size * Math.sin(angle);
    if (i === 0) {
      ctx.moveTo(px, py);
    } else {
      ctx.lineTo(px, py);
    }
  }
  ctx.closePath();
  ctx.fillStyle = fill;
  ctx.fill();
  ctx.strokeStyle = stroke;
  ctx.lineWidth = 1;
  ctx.stroke();
}

window.addEventListener("resize", renderActive);

world.generate_bsp(Date.now() >>> 0);
renderActive();
