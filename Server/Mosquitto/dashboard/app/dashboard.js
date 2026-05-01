const MAIN_CHART_COLOR = "#3b82f6";
const CHART_DISPLAY_WINDOW = 100;
const MAX_POINTS_IN_CHART = 500;
const CHART_UPDATE_INTERVAL_IN_MILLISECONDS = 5000;

class MosquittoDashboard {
  constructor() {
    this.brokerOnline = true;
    this.version = "";
    this.lastSysTopics = {};
    this.charts = {};
    this.chartData = {};
    this.timeoutHandler = null;

    this.initializeCharts();
    this.startDataUpdates();
  }

  setBrokerStatus() {
    const status = document.getElementById("broker-status");
    const statusText = document.getElementById("broker-status-text");
    if (this.brokerOnline) {
      status.classList.remove("bg-red-500");
      status.classList.add("bg-green-500");
      statusText.textContent = "Online";
    } else {
      status.classList.remove("bg-green-500");
      status.classList.add("bg-red-500");
      statusText.textContent = "Offline";
    }
  }

  setBrokerVersion() {
    this.updateHtmlElement("broker-version", this.version);
  }

  updateHtmlElement(elementId, value) {
    const element = document.getElementById(elementId);
    if (element) {
      element.textContent = value;
    }
  }

  createLineChart(canvasId, label) {
    const canvasElement = document.getElementById(canvasId);
    if (!canvasElement) return null;

    const ctx = canvasElement.getContext("2d");
    const chart = new Chart(ctx, {
      type: "line",
      data: {
        labels: this.chartData[canvasId]?.labels || [],
        datasets: [
          {
            label: label,
            data: this.chartData[canvasId]?.data || [],
            borderColor: MAIN_CHART_COLOR,
            backgroundColor: MAIN_CHART_COLOR + "30",
            borderWidth: 2,
            fill: true,
            tension: 0.3,
            pointRadius: 2,
            pointHoverRadius: 4,
          },
        ],
      },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        plugins: {
          legend: {
            display: false,
          },
        },
        scales: {
          x: {
            display: true,
            grid: { color: "#f3f4f6", drawBorder: false },
            ticks: { font: { size: 10 } },
          },
          y: {
            display: true,
            beginAtZero: true,
            grid: { color: "#f3f4f6", drawBorder: false },
            ticks: { font: { size: 10 } },
          },
        },
        interaction: {
          intersect: false,
          mode: "index",
        },
      },
    });
    return chart;
  }

  initializeCharts() {
    // Initialize chart data structures
    const chartIds = [
      "chart-clients-connected",
      "chart-messages-sent",
      "chart-messages-received",
      "chart-messages-dropped",
    ];

    chartIds.forEach((id) => {
      this.chartData[id] = { labels: [], data: [] };
    });

    // Create charts
    this.charts["chart-clients-connected"] = this.createLineChart(
      "chart-clients-connected",
      "Clients Online"
    );
    this.charts["chart-messages-sent"] = this.createLineChart(
      "chart-messages-sent",
      "Messages Sent"
    );
    this.charts["chart-messages-received"] = this.createLineChart(
      "chart-messages-received",
      "Messages Received"
    );
    this.charts["chart-messages-dropped"] = this.createLineChart(
      "chart-messages-dropped",
      "Messages Dropped"
    );
  }

  updateChart(chartId, datapoint, timestamp) {
    if (!this.charts[chartId] || !this.chartData[chartId]) return;

    const data = this.chartData[chartId];
    const timeString = new Date(timestamp).toLocaleTimeString();

    data.labels.push(timeString);
    data.data.push(datapoint);

    // Trim old data
    if (data.labels.length > MAX_POINTS_IN_CHART) {
      data.labels.shift();
      data.data.shift();
    }

    // Keep only last N points visible
    const start = Math.max(0, data.labels.length - CHART_DISPLAY_WINDOW);
    this.charts[chartId].data.labels = data.labels.slice(start);
    this.charts[chartId].data.datasets[0].data = data.data.slice(start);
    this.charts[chartId].update("none");
  }

  async checkForDataUpdates() {
    const timestamp = new Date().getTime();
    try {
      const response = await fetch("/api/status", {
        cache: "no-store",
      });
      const sysTopics = await response.json();

      this.brokerOnline = true;
      this.setBrokerStatus();

      if (sysTopics.version) {
        this.version = sysTopics.version;
        this.setBrokerVersion();
      }

      // Update metrics
      const updates = {
        "clients-connected": sysTopics["$SYS/broker/clients/connected"],
        "clients-disconnected": sysTopics["$SYS/broker/clients/disconnected"],
        "clients-total": sysTopics["$SYS/broker/clients/total"],
        "total-subscriptions": sysTopics["$SYS/broker/subscriptions/count"],
        "systopic-messages-received": sysTopics["$SYS/broker/messages/received"],
        "systopic-messages-sent": sysTopics["$SYS/broker/messages/sent"],
        "systopic-heap-current": sysTopics["$SYS/broker/heap/current"],
        "systopic-heap-max": sysTopics["$SYS/broker/heap/maximum"],
        "systopic-connection-sockets":
          sysTopics["$SYS/broker/connections/socket/count"],
        "systopic-messages-retained":
          sysTopics["$SYS/broker/retained messages/count"],
        "systopic-messages-stored": sysTopics["$SYS/broker/messages/stored"],
        "systopic-messages-received-1min":
          sysTopics["$SYS/broker/load/messages/received/1min"],
        "systopic-messages-received-10min":
          sysTopics["$SYS/broker/load/messages/received/10min"],
        "systopic-messages-received-15min":
          sysTopics["$SYS/broker/load/messages/received/15min"],
        "systopic-messages-sent-1min":
          sysTopics["$SYS/broker/load/messages/sent/1min"],
        "systopic-messages-sent-10min":
          sysTopics["$SYS/broker/load/messages/sent/10min"],
        "systopic-messages-sent-15min":
          sysTopics["$SYS/broker/load/messages/sent/15min"],
      };

      Object.entries(updates).forEach(([id, value]) => {
        if (value !== undefined && value !== this.lastSysTopics[id]) {
          this.updateHtmlElement(id, value);
          this.lastSysTopics[id] = value;
        }
      });

      // Update charts
      if (sysTopics["$SYS/broker/clients/connected"] !== undefined) {
        this.updateChart(
          "chart-clients-connected",
          sysTopics["$SYS/broker/clients/connected"],
          timestamp
        );
      }
      if (sysTopics["$SYS/broker/messages/sent"] !== undefined) {
        this.updateChart(
          "chart-messages-sent",
          sysTopics["$SYS/broker/messages/sent"],
          timestamp
        );
      }
      if (sysTopics["$SYS/broker/messages/received"] !== undefined) {
        this.updateChart(
          "chart-messages-received",
          sysTopics["$SYS/broker/messages/received"],
          timestamp
        );
      }
      if (sysTopics["$SYS/broker/publish/messages/dropped"] !== undefined) {
        this.updateChart(
          "chart-messages-dropped",
          sysTopics["$SYS/broker/publish/messages/dropped"],
          timestamp
        );
      }
    } catch (error) {
      this.brokerOnline = false;
      this.setBrokerStatus();
    }
  }

  async startDataUpdates() {
    const update = async () => {
      try {
        await this.checkForDataUpdates();
      } catch (error) {
        console.error("Error updating data:", error);
      }

      this.timeoutHandler = setTimeout(
        () => update(),
        CHART_UPDATE_INTERVAL_IN_MILLISECONDS
      );
    };

    update();
  }
}

// Initialize on load
document.addEventListener("DOMContentLoaded", () => {
  new MosquittoDashboard();
});
