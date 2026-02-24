/**
 * Tension Controller Dashboard JavaScript
 * Uses HTTP polling for real-time data updates
 */

const MAX_CHART_POINTS = 100;
const POLL_INTERVAL_MS = 500;

// Chart data
const chartData = {
    labels: [],
    tension: [],
    speed: [],
    pwm: []
};

// State names
const STATES = ['Idle', 'Starting', 'Running', 'Stopping', 'Warning', 'Fault', 'E-Stop'];

// Command enum matching C code
const CMD = {
    START: 0,
    STOP: 1,
    ESTOP: 2,
    SET_TENSION: 3,
    SET_SPEED: 4,
    TARE: 5,
    CALIBRATE: 6,
    AUTOTUNE_SPEED: 7,
    AUTOTUNE_TENSION: 8,
    RESET_FAULTS: 9,
    SAVE_CONFIG: 10
};

// Initialize Chart.js
let liveChart;
function initChart() {
    const ctx = document.getElementById('liveChart').getContext('2d');
    liveChart = new Chart(ctx, {
        type: 'line',
        data: {
            labels: chartData.labels,
            datasets: [
                {
                    label: 'Tension (kg)',
                    data: chartData.tension,
                    borderColor: '#3b82f6',
                    backgroundColor: 'rgba(59, 130, 246, 0.1)',
                    tension: 0.3,
                    fill: true
                },
                {
                    label: 'Speed (RPM/10)',
                    data: chartData.speed,
                    borderColor: '#10b981',
                    backgroundColor: 'rgba(16, 185, 129, 0.1)',
                    tension: 0.3,
                    fill: true
                },
                {
                    label: 'PWM (%)',
                    data: chartData.pwm,
                    borderColor: '#f59e0b',
                    backgroundColor: 'rgba(245, 158, 11, 0.1)',
                    tension: 0.3,
                    fill: true
                }
            ]
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            animation: { duration: 0 },
            scales: {
                x: {
                    display: false
                },
                y: {
                    beginAtZero: true,
                    grid: { color: 'rgba(148, 163, 184, 0.1)' },
                    ticks: { color: '#94a3b8' }
                }
            },
            plugins: {
                legend: {
                    labels: { color: '#f8fafc' }
                }
            }
        }
    });
}

// Update dashboard with new data
function updateDashboard(data) {
    // Update connection status
    document.getElementById('connection-status').className = 'status-indicator connected';

    // Update cards
    document.getElementById('tension-value').textContent = data.tension?.toFixed(2) ?? '--';
    document.getElementById('tension-sp').textContent = data.tensionSP?.toFixed(2) ?? '--';
    document.getElementById('speed-value').textContent = data.speed?.toFixed(0) ?? '--';
    document.getElementById('speed-sp').textContent = data.speedSP?.toFixed(0) ?? '--';
    document.getElementById('pwm-value').textContent = data.pwm?.toFixed(1) ?? '--';
    document.getElementById('state-value').textContent = STATES[data.state] ?? 'Unknown';

    // PWM progress bar
    const pwmBar = document.getElementById('pwm-bar');
    pwmBar.style.width = `${Math.min(100, Math.max(0, data.pwm || 0))}%`;

    // Uptime
    if (data.uptime !== undefined) {
        const hrs = Math.floor(data.uptime / 3600);
        const mins = Math.floor((data.uptime % 3600) / 60);
        const secs = data.uptime % 60;
        document.getElementById('uptime').textContent =
            `${hrs.toString().padStart(2, '0')}:${mins.toString().padStart(2, '0')}:${secs.toString().padStart(2, '0')}`;
    }

    // State card color
    const stateCard = document.querySelector('.state-card');
    stateCard.classList.remove('state-running', 'state-warning', 'state-fault');
    if (data.state === 2) stateCard.classList.add('state-running');
    else if (data.state === 4) stateCard.classList.add('state-warning');
    else if (data.state >= 5) stateCard.classList.add('state-fault');

    // Faults
    const faultSection = document.getElementById('fault-section');
    const faultList = document.getElementById('fault-list');
    if (data.faults && data.faults > 0) {
        faultSection.classList.remove('hidden');
        faultList.innerHTML = getFaultStrings(data.faults).map(f => `<p>⚠️ ${f}</p>`).join('');
    } else {
        faultSection.classList.add('hidden');
    }

    // Update chart
    updateChart(data);
}

function getFaultStrings(faultFlags) {
    const faults = [];
    if (faultFlags & 0x0001) faults.push('Tension Over Limit');
    if (faultFlags & 0x0002) faults.push('Speed Over Limit');
    if (faultFlags & 0x0004) faults.push('Motor Stall');
    if (faultFlags & 0x0008) faults.push('Encoder Failure');
    if (faultFlags & 0x0010) faults.push('Load Cell Failure');
    if (faultFlags & 0x0020) faults.push('Watchdog Timeout');
    if (faultFlags & 0x8000) faults.push('Emergency Stop');
    return faults;
}

function updateChart(data) {
    const now = new Date().toLocaleTimeString();

    chartData.labels.push(now);
    chartData.tension.push(data.tension || 0);
    chartData.speed.push((data.speed || 0) / 10);  // Scale down for display
    chartData.pwm.push(data.pwm || 0);

    // Limit data points
    if (chartData.labels.length > MAX_CHART_POINTS) {
        chartData.labels.shift();
        chartData.tension.shift();
        chartData.speed.shift();
        chartData.pwm.shift();
    }

    liveChart.update('none');
}

// Fetch status via HTTP
function fetchStatus() {
    fetch('/api/status')
        .then(res => res.json())
        .then(updateDashboard)
        .catch(() => {
            document.getElementById('connection-status').className = 'status-indicator disconnected';
        });
}

// Send command via REST API
function sendCommand(cmd, value = 0) {
    fetch('/api/command', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ cmd, value })
    }).catch(err => console.error('Command failed:', err));
}

// Button handlers
document.getElementById('btn-start').addEventListener('click', () => sendCommand(CMD.START));
document.getElementById('btn-stop').addEventListener('click', () => sendCommand(CMD.STOP));
document.getElementById('btn-estop').addEventListener('click', () => {
    if (confirm('Trigger Emergency Stop?')) {
        sendCommand(CMD.ESTOP);
    }
});

document.getElementById('btn-set').addEventListener('click', () => {
    const value = parseFloat(document.getElementById('tension-input').value);
    if (!isNaN(value) && value >= 0) {
        sendCommand(CMD.SET_TENSION, value);
    }
});

document.getElementById('btn-tare').addEventListener('click', () => sendCommand(CMD.TARE));
document.getElementById('btn-autotune').addEventListener('click', () => {
    if (confirm('Start speed loop auto-tuning? System will oscillate briefly.')) {
        sendCommand(CMD.AUTOTUNE_SPEED);
    }
});
document.getElementById('btn-reset').addEventListener('click', () => sendCommand(CMD.RESET_FAULTS));

// Initialize
document.addEventListener('DOMContentLoaded', () => {
    initChart();

    // Start HTTP polling
    fetchStatus();
    setInterval(fetchStatus, POLL_INTERVAL_MS);
});
