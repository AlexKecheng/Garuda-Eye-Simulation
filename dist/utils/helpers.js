"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.formatData = exports.calculateDistance = void 0;
function calculateDistance(pointA, pointB) {
    const dx = pointB.x - pointA.x;
    const dy = pointB.y - pointA.y;
    return Math.sqrt(dx * dx + dy * dy);
}
exports.calculateDistance = calculateDistance;
function formatData(data) {
    return `ID: ${data.id}, Timestamp: ${new Date(data.timestamp).toISOString()}, Value: ${data.value}`;
}
exports.formatData = formatData;
