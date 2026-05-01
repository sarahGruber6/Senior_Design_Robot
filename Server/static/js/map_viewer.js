class MapViewer {
    constructor(canvasId) {
        this.mapCanvas  = new MapCanvas(canvasId);
        this.isRunning  = false;

        this.currentPose = { x_mm: 0, y_mm: 0, theta_deg: 0 };
        this.plannedPath = null;
        this.currentGoal = null;
        this.mapBuffer   = null;
        this.places      = [];

        this.mapUpdateInterval    = 500;
        this.poseUpdateInterval   = 200;
        this.pathUpdateInterval   = 300;
        this.placesUpdateInterval = 5000;

        this._setupMouseMove();
    }

    start() {
        if(this.isRunning) return;
        this.isRunning = true;
        this._poseLoop();
        this._mapLoop();
        this._pathLoop();
        this._placesLoop();
    }

    stop() {
        this.isRunning = false;
    }

    _setupMouseMove() {
        this.mapCanvas.canvas.addEventListener('mousemove', e => {
            const rect = this.mapCanvas.canvas.getBoundingClientRect();
            const [mmX, mmY] = this.mapCanvas.screenToMap(
                e.clientX - rect.left,
                e.clientY - rect.top,
            );
            const el = document.getElementById('coordDisplay');
            if(el) el.textContent = `(${Math.round(mmX)}, ${Math.round(mmY)}) mm`;
        });
    }

    async _poseLoop() {
        while(this.isRunning) {
            try {
                const res = await fetch('/api/slam/pose');
                if(res.ok) {
                    const data = await res.json();
                    if(data.ok) {
                        this.currentPose = { x_mm: data.x_mm, y_mm: data.y_mm, theta_deg: data.theta_deg };
                        this._render();
                    }
                }
            } catch (e) { console.error('Pose error:', e); }
            await this._sleep(this.poseUpdateInterval);
        }
    }

    async _mapLoop() {
        while(this.isRunning) {
            try {
                const res = await fetch('/api/slam/map');
                if(res.ok) {
                    const data = await res.json();
                    if(data.ok) {
                        const bin = atob(data.data);
                        const bytes = new Uint8Array(bin.length);
                        for(let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);

                        // If the server reports a different map resolution, update to match
                        if(data.width && data.width !== this.mapCanvas.mapSizePixels) {
                            this.mapCanvas.mapSizePixels = data.width;
                            this.mapCanvas.canvas.width  = data.width;
                            this.mapCanvas.canvas.height = data.height || data.width;
                            this.mapCanvas.pixelPerMm    = data.width  / this.mapCanvas.mapSizeMm;
                            this.mapCanvas.mmPerPixel    = this.mapCanvas.mapSizeMm / data.width;
                        }

                        this.mapBuffer = bytes;
                        this._render();
                    }
                }
            } catch (e) { console.error('Map error:', e); }
            await this._sleep(this.mapUpdateInterval);
        }
    }

    async _pathLoop() {
        while(this.isRunning) {
            try {
                const res = await fetch('/api/autonomy/path');
                if(res.ok) {
                    const data = await res.json();
                    if(data.ok) {
                        this.plannedPath = data.path;
                        this.currentGoal = data.goal;
                        this._render();
                    }
                }
            } catch (e) { console.error('Path error:', e); }
            await this._sleep(this.pathUpdateInterval);
        }
    }

    async _placesLoop() {
        while(this.isRunning) {
            try {
                const res = await fetch('/api/places');
                if(res.ok) {
                    const data = await res.json();
                    if(data.ok) {
                        this.places = data.places || [];
                        this._render();
                    }
                }
            } catch (e) { console.error('Places error:', e); }
            await this._sleep(this.placesUpdateInterval);
        }
    }

    _render() {
        if(!this.mapBuffer) {
            this.mapCanvas.clearBackground();
            const colors = this.mapCanvas.getColors();
            const ctx    = this.mapCanvas.ctx;
            const w      = this.mapCanvas.canvas.width;
            const h      = this.mapCanvas.canvas.height;
            ctx.fillStyle    = colors.text;
            ctx.font         = `${Math.round(w / 40)}px sans-serif`;
            ctx.textAlign    = 'center';
            ctx.textBaseline = 'middle';
            ctx.fillText('Waiting for LiDAR data...', w / 2, h / 2);
            return;
        }

        this.mapCanvas.clearBackground();
        this.mapCanvas.drawOccupancyGrid(this.mapBuffer);

        if(this.places.length) this.mapCanvas.drawPlaces(this.places);
        if(this.plannedPath)   this.mapCanvas.drawPath(this.plannedPath);
        if(this.currentGoal)   this.mapCanvas.drawGoal(this.currentGoal[0], this.currentGoal[1]);

        this.mapCanvas.drawRobot(
            this.currentPose.x_mm,
            this.currentPose.y_mm,
            this.currentPose.theta_deg,
        );
    }

    _sleep(ms) { return new Promise(r => setTimeout(r, ms)); }
}
