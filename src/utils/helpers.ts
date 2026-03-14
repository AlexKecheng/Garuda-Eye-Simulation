export function calculateDistance(pointA: { x: number; y: number }, pointB: { x: number; y: number }): number {
    const dx = pointB.x - pointA.x;
    const dy = pointB.y - pointA.y;
    return Math.sqrt(dx * dx + dy * dy);
}

export function formatData(data: { id: number; timestamp: number; value: number }): string {
    return `ID: ${data.id}, Timestamp: ${new Date(data.timestamp).toISOString()}, Value: ${data.value}`;
}