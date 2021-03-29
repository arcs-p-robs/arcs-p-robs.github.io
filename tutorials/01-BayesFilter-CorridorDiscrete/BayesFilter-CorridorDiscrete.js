// user set motion model values
// user set sensor model values
// user set position
// generate corridors (have some that are canned, easy/medium/hard)
// css on d3
// https://www.redblobgames.com/making-of/line-drawing/

// ENVIRONMENT
const WALL = 0;
const DOOR = 1;
let corridor;

// STATE
let trueLocation;
let locationBeliefs;
let locationBeliefsBar;

// MOTION MODEL
// The robot can attempt to move one cell at a time, and
// it tries to move every time step.
let moveSuccessByWall; // p(Xt = New  | U = Move, Xt-1 = Wall)
let moveFailureByWall; // p(Xt = Xt-1 | U = Move, Xt-1 = Wall)
let moveSuccessByDoor; // p(Xt = New  | U = Move, Xt-1 = Door)
let moveFailureByDoor; // p(Xt = Xt-1 | U = Move, Xt-1 = Door)

// SENSOR MODEL
// The robot can sense the wall/door
let senseWallByWall; // p(Z = Wall | X = Wall)
let senseDoorByWall; // p(Z = Door | X = Wall)
let senseDoorByDoor; // p(Z = Wall | X = Door)
let senseWallByDoor; // p(Z = Door | X = Door)

// USER INTERFACE
let resetButton;
let stepButton;

// DISPLAY
let viewWidth;
let viewHeight;
let margin;
let xPosition;
let barHeight;
let svg;

initUI();
initSim();
initDisp();
draw();

function getRandomInt(max) {
    return Math.floor(Math.random() * Math.floor(max));
}

function createArrayWithXofY(x, y) {
    const newArray = [];
    for (let index = 0; index < x; index++) {
        newArray.push(y);
    }
    return newArray;
}

function scaleArrayToProbs(array) {
    let sum = 0;
    for (const val of array) {
        sum += val;
    }

    for (let i = 0; i < array.length; i++) {
        array[i] /= sum;
    }
}

function initUI() {
    resetButton = document.getElementById("reset-button");
    resetButton.addEventListener("click", reset);

    stepButton = document.getElementById("step-button");
    stepButton.addEventListener("click", step);
}


function initSim() {
    // ENVIRONMENT
    corridor = [
        WALL, WALL, DOOR, WALL, DOOR,
        WALL, DOOR, DOOR, WALL, WALL,
    ];

    // MOTION MODEL
    moveSuccessByWall = 1.0;
    moveFailureByWall = 1.0 - moveSuccessByWall;
    moveSuccessByDoor = 1.0;
    moveFailureByDoor = 1.0 - moveSuccessByDoor;

    // SENSOR MODEL
    senseWallByWall = 0.8;
    senseDoorByWall = 1.0 - senseWallByWall;
    senseDoorByDoor = 0.8;
    senseWallByDoor = 1.0 - senseDoorByDoor;

    // STATE
    trueLocation = getRandomInt(corridor.length);
    locationBeliefs = createArrayWithXofY(corridor.length, 1.0 / corridor.length);
    locationBeliefsBar = createArrayWithXofY(corridor.length, 0.0);
}

function initDisp() {

    viewWidth = 500;
    viewHeight = 200;
    margin = { top: 2, right: 2, bottom: 20, left: 2 };

    const corridorHeight = 100;

    xPosition = d3.scaleBand()
        .range([margin.left, viewWidth - margin.right])
        .domain([...locationBeliefs.keys()])
        .paddingInner(0.05);

    const xAxis = d3.axisTop(xPosition);

    barHeight = d3.scaleLinear()
        .range([margin.top, viewHeight - margin.bottom - margin.top - corridorHeight])
        .domain([0, 1]);

    svg = d3.select("#display")
        .attr("viewBox", `0 0 ${viewWidth} ${viewHeight}`);

    svg.append("g")
        .attr("transform", `translate(0, ${viewHeight})`)
        .call(xAxis);

    const cells = svg.selectAll(".cells")
        .data(corridor);

    cells.enter()
        .append("rect")
        .attr("class", "cells")
        .attr("x", function (_, i) { return xPosition(i); })
        .attr("y", margin.top)
        .attr("width", xPosition.bandwidth())
        .attr("height", corridorHeight / 2)
        .attr("fill", function (d) { return d == WALL ? "#DDD" : "firebrick"; })
        .attr("stroke", "gray");

    svg.append("rect")
        .attr("id", "robot")
        .attr("y", corridorHeight / 2 - 10)
        .attr("x", xPosition(trueLocation) + 5)
        .attr("width", 20)
        .attr("height", 20)
        .attr("fill", "lightblue")
        .attr("stroke", "orange");
}

function reset() {
    trueLocation = getRandomInt(corridor.length);
    locationBeliefs = createArrayWithXofY(corridor.length, 1.0 / corridor.length);
    locationBeliefsBar = createArrayWithXofY(corridor.length, 0.0);

    draw();
}


function step() {
    console.log("Stepping");
    let movedThisTurn = false;
    let sensedThisTurn = false;

    motionStep();
    sensorStep();
    draw();
}

function motionStep() {
    // Move the robot
    const trueMoveProb = corridor[trueLocation] == WALL ? moveSuccessByWall : moveSuccessByDoor;
    if (trueMoveProb > Math.random()) {
        movedThisTurn = true;
        trueLocation = (trueLocation + 1) % corridor.length;
    }

    // Update locationBeliefsBar based on motion model
    for (let loc = 0; loc < corridor.length; loc++) {
        locationBeliefsBar[loc] = 0;

        // Update our belief about this location based on two possibilities
        // (1) we were in the preceding location and successfully moved here
        const precLoc = loc == 0 ? corridor.length - 1 : loc - 1;
        const moveProb = corridor[precLoc] == WALL ? moveSuccessByWall : moveSuccessByDoor;
        locationBeliefsBar[loc] += locationBeliefs[precLoc] * moveProb;

        // (2) we failed to move from here to the next location
        const failProb = corridor[loc] == WALL ? moveFailureByWall : moveFailureByDoor;
        locationBeliefsBar[loc] += locationBeliefs[loc] * failProb;
    }
}

function sensorStep() {
    // Sense the environment
    const trueWallSenseProb = corridor[trueLocation] == WALL ? senseWallByWall : senseWallByDoor;
    const sensedEnv = trueWallSenseProb > Math.random() ? WALL : DOOR;
    const trueEnv = corridor[trueLocation];
    sensedThisTurn = sensedEnv == trueEnv;

    // Update locationBeliefs based on sensor model
    for (let loc = 0; loc < corridor.length; loc++) {
        const senseProb = corridor[loc] == WALL ? senseWallByWall : senseDoorByDoor;
        const scale = senseProb / (1 - senseProb);
        const likelihood = corridor[loc] == sensedEnv ? scale : 1;
        locationBeliefs[loc] = likelihood * locationBeliefsBar[loc];
    }
    scaleArrayToProbs(locationBeliefs);
}

function draw() {

    const bars = svg.selectAll(".bars")
        .data(locationBeliefs, function (_, i) { return i; });

    bars.exit()
        .transition()
        .duration(500)
        .attr("height", 0)
        .remove();

    bars.enter()
        .append("rect")
        .attr("class", "bars")
        .attr("x", function (_, i) { return xPosition(i); })
        .attr("y", 0)
        .attr("width", xPosition.bandwidth())
        .attr("height", 0)
        .attr("fill", "hotpink")
        .merge(bars).transition()
        .duration(500)
        .delay(100)
        .attr("y", function (d) { return viewHeight - margin.bottom - barHeight(d); })
        .attr("height", function (d) { return barHeight(d); });

    svg.select("#robot")
        .transition()
        .duration(500)
        .attr("x", xPosition(trueLocation) + 5);
}
