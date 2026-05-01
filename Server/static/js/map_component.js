class MapCanvas {
    constructor(canvasId, mapSizeMm = 20000, mapSizePixels = 800) {
        this.canvas       = document.getElementById(canvasId);
        this.ctx          = this.canvas.getContext('2d');
        this.mapSizeMm    = mapSizeMm;
        this.mapSizePixels = mapSizePixels;
        this.flipY        = true;

        // Set canvas logical resolution to match the map data exactly.
        // CSS (width: 100%; height: auto) handles display scaling separately.
        this.canvas.width  = mapSizePixels;
        this.canvas.height = mapSizePixels;

        // mm <-> map-pixel ratios (used for drawing only, not for click math)
        this.mmPerPixel  = mapSizeMm / mapSizePixels;
        this.pixelPerMm  = mapSizePixels / mapSizeMm;

        this.isDarkTheme = document.documentElement.getAttribute('data-theme') === 'dark';

        const observer = new MutationObserver(() => {
            this.isDarkTheme = document.documentElement.getAttribute('data-theme') === 'dark';
        });
        observer.observe(document.documentElement, { attributes: true });
    }

    getColors() {
        if(this.isDarkTheme){
            return {
                background: '#151515',
                freespace:  '#1a1a1a',
                occupied:   '#ff4444',
                dynamic:    '#3399ff',   // transient obstacle (person walking by)
                unknown:    '#333333',
                robot:      '#00ff88',
                goal:       '#ffcc00',
                path:       '#00ccff',
                text:       '#ffffff',
            };
        }
        return {
            background: '#ffffff',
            freespace:  '#f5f5f5',
            occupied:   '#333333',
            dynamic:    '#2277ee',       // transient obstacle (person walking by)
            unknown:    '#cccccc',
            robot:      '#00aa44',
            goal:       '#ff6600',
            path:       '#0066ff',
            text:       '#000000',
        };
    }

    clearBackground() {
        const colors = this.getColors();
        this.ctx.save();
        this.ctx.setTransform(1, 0, 0, 1, 0, 0);
        this.ctx.fillStyle = colors.background;
        this.ctx.fillRect(0, 0, this.canvas.width, this.canvas.height);
        this.ctx.restore();
    }

    drawOccupancyGrid(mapBytes) {
        const colors    = this.getColors();
        const imageData = this.ctx.createImageData(this.mapSizePixels, this.mapSizePixels);
        const data      = imageData.data;

        for(let i = 0; i < mapBytes.length; i++) {
            const srcX = i % this.mapSizePixels;
            const srcY = Math.floor(i / this.mapSizePixels);
            const dstY = this.flipY ? (this.mapSizePixels - 1 - srcY) : srcY;
            const dstI = (dstY * this.mapSizePixels + srcX) * 4;
            const v = mapBytes[i];
            let [r, g, b] = v > 200  ? this._hex3(colors.freespace)  // free space
                          : v >= 150 ? this._hex3(colors.dynamic)    // transient obstacle
                          : v < 50   ? this._hex3(colors.occupied)   // wall / obstacle
                                     : this._hex3(colors.unknown);   // unexplored
            data[dstI]     = r;
            data[dstI + 1] = g;
            data[dstI + 2] = b;
            data[dstI + 3] = 255;
        }

        this.ctx.putImageData(imageData, 0, 0);
    }

    drawRobot(xMm, yMm, thetaDeg) {
        const colors = this.getColors();
        const [x, y] = this.mapToCanvas(xMm, yMm);
        const rad    = (this.flipY ? -thetaDeg : thetaDeg) * Math.PI / 180;

        this.ctx.save();
        this.ctx.fillStyle = colors.robot;
        this.ctx.beginPath();
        this.ctx.arc(x, y, 15, 0, 2 * Math.PI);
        this.ctx.fill();

        this.ctx.strokeStyle = colors.robot;
        this.ctx.lineWidth   = 3;
        this.ctx.beginPath();
        this.ctx.moveTo(x, y);
        this.ctx.lineTo(x + 20 * Math.cos(rad), y + 20 * Math.sin(rad));
        this.ctx.stroke();
        this.ctx.restore();
    }

    drawGoal(xMm, yMm) {
        const colors = this.getColors();
        const [x, y] = this.mapToCanvas(xMm, yMm);

        this.ctx.save();
        this.ctx.fillStyle = colors.goal;
        this._drawStar(x, y, 5, 12, 6);
        this.ctx.restore();
    }

    drawPath(pathWaypoints) {
        if(!pathWaypoints || pathWaypoints.length < 2) return;
        const colors = this.getColors();

        this.ctx.save();
        this.ctx.strokeStyle = colors.path;
        this.ctx.lineWidth   = 2;
        this.ctx.setLineDash([5, 5]);

        this.ctx.beginPath();
        let [x, y] = this.mapToCanvas(pathWaypoints[0][0], pathWaypoints[0][1]);
        this.ctx.moveTo(x, y);
        for(let i = 1; i < pathWaypoints.length; i++) {
            [x, y] = this.mapToCanvas(pathWaypoints[i][0], pathWaypoints[i][1]);
            this.ctx.lineTo(x, y);
        }
        this.ctx.stroke();
        this.ctx.restore();
    }

    screenToMap(cssX, cssY) {
        const rect = this.canvas.getBoundingClientRect();
        const yRatio = cssY / rect.height;
        return [
            (cssX / rect.width)  * this.mapSizeMm,
            (this.flipY ? (1 - yRatio) : yRatio) * this.mapSizeMm,
        ];
    }

    mapToCanvas(xMm, yMm) {
        const x = xMm * this.pixelPerMm;
        const yRaw = yMm * this.pixelPerMm;
        const y = this.flipY ? (this.mapSizePixels - yRaw) : yRaw;
        return [x, y];
    }

    drawPlaces(places) {
        if(!places || !places.length) return;
        const colors  = this.getColors();
        const fontSize = Math.max(10, Math.round(this.mapSizePixels / 70));

        for(const p of places) {
            const [x, y] = this.mapToCanvas(p.x_mm, p.y_mm);

            this.ctx.save();

            // Pin circle
            this.ctx.beginPath();
            this.ctx.arc(x, y, 7, 0, 2 * Math.PI);
            this.ctx.fillStyle   = colors.goal;
            this.ctx.strokeStyle = colors.background;
            this.ctx.lineWidth   = 2;
            this.ctx.fill();
            this.ctx.stroke();

            // Label
            this.ctx.font         = `bold ${fontSize}px sans-serif`;
            this.ctx.textAlign    = 'left';
            this.ctx.textBaseline = 'middle';
            this.ctx.fillStyle    = colors.text;
            this.ctx.fillText(p.name, x + 10, y);

            this.ctx.restore();
        }
    }

    _drawStar(cx, cy, spikes, outer, inner) {
        let rot = Math.PI / 2 * 3;
        const step = Math.PI / spikes;
        this.ctx.beginPath();
        this.ctx.moveTo(cx, cy - outer);
        for(let i = 0; i < spikes; i++) {
            this.ctx.lineTo(cx + Math.cos(rot) * outer, cy + Math.sin(rot) * outer);
            rot += step;
            this.ctx.lineTo(cx + Math.cos(rot) * inner, cy + Math.sin(rot) * inner);
            rot += step;
        }
        this.ctx.closePath();
        this.ctx.fill();
    }

    _hex3(hex) {
        const r = /^#?([a-f\d]{2})([a-f\d]{2})([a-f\d]{2})$/i.exec(hex);
        return r ? [parseInt(r[1], 16), parseInt(r[2], 16), parseInt(r[3], 16)] : [0, 0, 0];
    }
}
