document.addEventListener('DOMContentLoaded', () => {
    const frequencySlider = document.getElementById('frequency');
    const sigmaSlider = document.getElementById('sigma');
    const resolutionSlider = document.getElementById('resolution');

    const frequencyValueSpan = document.getElementById('frequency-value');
    const sigmaValueSpan = document.getElementById('sigma-value');
    const resolutionValueSpan = document.getElementById('resolution-value');

    const plotMorletWavelet3D = () => {
        const frequency = parseFloat(frequencySlider.value);
        const sigma = parseFloat(sigmaSlider.value);
        const resolution = parseInt(resolutionSlider.value);

        frequencyValueSpan.textContent = frequency;
        sigmaValueSpan.textContent = sigma;
        resolutionValueSpan.textContent = resolution;

        const numPoints = resolution;
        const time_range = 4.0;
        const time_step = time_range / numPoints;

        const x = [];
        const y = [];
        const z = [];

        for (let t = -2; t <= 2; t += time_step) {
            const gaussian = Math.exp(-t * t / (2 * sigma * sigma));
            const real = gaussian * Math.cos(2 * Math.PI * frequency * t);
            const imag = gaussian * Math.sin(2 * Math.PI * frequency * t);

            x.push(t);
            y.push(real);
            z.push(imag);
        }

        const data = [{
            x: x,
            y: y,
            z: z,
            type: 'scatter3d',
            mode: 'lines',
            line: { color: 'blue', width: 2 }
        }];

        const layout = {
            title: '3D Complex Morlet Wavelet',
            scene: {
                xaxis: { title: 'Time (s)' },
                yaxis: { title: 'Real Part' },
                zaxis: { title: 'Imaginary Part' }
            },
            height: 600
        };

        Plotly.newPlot('morlet-graph', data, layout);
    };

    const updatePlot = () => {
        const frequency = parseFloat(frequencySlider.value);
        const sigma = parseFloat(sigmaSlider.value);
        const resolution = parseInt(resolutionSlider.value);

        frequencyValueSpan.textContent = frequency;
        sigmaValueSpan.textContent = sigma;
        resolutionValueSpan.textContent = resolution;

        const numPoints = resolution;
        const time_range = 4.0;
        const time_step = time_range / numPoints;

        const x = [];
        const y = [];
        const z = [];

        for (let t = -2; t <= 2; t += time_step) {
            const gaussian = Math.exp(-t * t / (2 * sigma * sigma));
            const real = gaussian * Math.cos(2 * Math.PI * frequency * t);
            const imag = gaussian * Math.sin(2 * Math.PI * frequency * t);

            x.push(t);
            y.push(real);
            z.push(imag);
        }

        const update = {
            x: [x],
            y: [y],
            z: [z]
        };

        Plotly.restyle('morlet-graph', update);
    };

    frequencySlider.addEventListener('input', updatePlot);
    sigmaSlider.addEventListener('input', updatePlot);
    resolutionSlider.addEventListener('input', updatePlot);

    plotMorletWavelet3D();
});