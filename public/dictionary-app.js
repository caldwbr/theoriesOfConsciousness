document.addEventListener('DOMContentLoaded', () => {
    const container = document.getElementById('dictionary-entries');
    if (!container) return;

    // --- 1. Sanitization Function ---
    const sanitizeNbsp = (root) => {
        const walker = document.createTreeWalker(root, NodeFilter.SHOW_TEXT);
        const nodes = [];
        while (walker.nextNode()) nodes.push(walker.currentNode);
        nodes.forEach(n => {
            n.nodeValue = n.nodeValue.replace(/&nbsp;?/gi, ' ').replace(/\u00A0+/g, ' ');
        });
        root.querySelectorAll('p').forEach(p => {
            if (/^[\s\u00A0]*$/.test(p.textContent)) p.remove();
        });
    };
    
    sanitizeNbsp(document.body);

    // --- 2. Core Routing, Sorting, and Filtering Logic ---
    const CATEGORIES = [
        'mainstream', 'ring-bank', 'michael-levin', 'predictive-processing',
        'neuroscience', 'person', 'math', 'immanuel-kant', 'dynamics', 'book', 'paper'
    ];

    const slugify = (s) => s.toLowerCase()
        .replace(/&/g, ' and ')
        .replace(/[’'"]/g, '')
        .replace(/[^a-z0-9\s-]/g, '')
        .replace(/\s+/g, '-')
        .replace(/-+/g, '-')
        .replace(/^-|-$/g, '');

    const getCategory = (entry) => CATEGORIES.find(c => entry.classList.contains(c)) || 'misc';

    const seen = new Map();
    const unique = (base) => {
        const n = seen.get(base) || 0;
        seen.set(base, n + 1);
        return n ? `${base}-${n+1}` : base;
    };

    const normalizeCards = () => {
        const entries = Array.from(container.querySelectorAll('.entry'));
        return entries.map(entry => {
            const titleEl = entry.querySelector('.entry-title');
            const title = titleEl ? titleEl.textContent.trim() : 'entry';
            const cat = getCategory(entry);
            const base = slugify(`${title} ${cat}`);
            const slug = unique(base);

            entry.id = slug;

            let card = entry.parentElement;
            if (!(card && card.classList && card.classList.contains('card-link'))) {
                card = document.createElement('a');
                card.className = 'card-link';
                entry.replaceWith(card);
                card.appendChild(entry);
            }
            card.setAttribute('href', `#${slug}`);
            card.style.display = 'block';
            return card;
        });
    };

    let cards = normalizeCards();

    const sortCards = () => {
        cards.sort((a, b) => {
            const ta = a.querySelector('.entry-title')?.textContent.trim() || '';
            const tb = b.querySelector('.entry-title')?.textContent.trim() || '';
            return ta.localeCompare(tb);
        });
        cards.forEach(c => container.appendChild(c));
    };
    sortCards();

    const showAll = () => {
        cards.forEach(c => c.style.display = 'block');
    };

    const applyFilter = (value) => {
        cards.forEach(c => {
            const entry = c.querySelector('.entry');
            const match = (value === 'all') || entry.classList.contains(value);
            c.style.display = match ? 'block' : 'none';
        });
    };

    // --- URL State Management ---
    const getCurrentFilter = () => {
        const urlParams = new URLSearchParams(window.location.search);
        return urlParams.get('filter') || 'all';
    };

    const updateURL = (filter, hash = '') => {
        const urlParams = new URLSearchParams();
        if (filter && filter !== 'all') {
            urlParams.set('filter', filter);
        }
        const queryString = urlParams.toString();
        const newURL = queryString ? 
            `${location.pathname}?${queryString}${hash}` : 
            `${location.pathname}${hash}`;
        
        history.pushState(null, '', newURL);
    };

    const filterButtons = document.querySelectorAll('.filter-btn');
    filterButtons.forEach(btn => {
        btn.addEventListener('click', () => {
            const filterValue = btn.getAttribute('data-filter');
            filterButtons.forEach(b => b.classList.remove('active'));
            btn.classList.add('active');
            applyFilter(filterValue);
            updateURL(filterValue); // Store filter in URL
        });
    });

    // --- 3. Morlet Wavelet Plot Logic ---
    let plotInitialized = false;

    const initializeMorletPlot = () => {
        const morletEntry = document.querySelector('#complex-morlet-wavelet-math');
        const morletCard = morletEntry ? morletEntry.closest('.card-link') : null;
        if (!morletCard) return;

        const frequencySlider = morletCard.querySelector('#frequency');
        const sigmaSlider = morletCard.querySelector('#sigma');
        const resolutionSlider = morletCard.querySelector('#resolution');

        if (!frequencySlider || plotInitialized) return;
        plotInitialized = true;

        const frequencyValueSpan = morletCard.querySelector('#frequency-value');
        const sigmaValueSpan = morletCard.querySelector('#sigma-value');
        const resolutionValueSpan = morletCard.querySelector('#resolution-value');

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
            const update = { x: [x], y: [y], z: [z] };
            Plotly.restyle('morlet-graph', update);
        };

        const initialPlot = () => {
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
                x: x, y: y, z: z, type: 'scatter3d',
                mode: 'lines', line: { color: 'blue', width: 2 }
            }];
            const layout = {
                title: '3D Complex Morlet Wavelet',
                scene: {
                    xaxis: { title: 'Time (s)' }, yaxis: { title: 'Real Part' },
                    zaxis: { title: 'Imaginary Part' }
                },
                height: 600
            };
            Plotly.newPlot('morlet-graph', data, layout);
        };
        
        const debounce = (func, delay) => {
            let timeoutId;
            return (...args) => {
                clearTimeout(timeoutId);
                timeoutId = setTimeout(() => func(...args), delay);
            };
        };

        frequencySlider.addEventListener('input', debounce(updatePlot, 200));
        sigmaSlider.addEventListener('input', debounce(updatePlot, 200));
        resolutionSlider.addEventListener('input', debounce(updatePlot, 200));
        initialPlot();
    };

    // --- 4. Lorenz Attractor Plot Logic ---
    let lorenzAnimationId = null;

    const initializeLorenzPlot = () => {
        const lorenzEntry = document.querySelector('#lorenz-attractor-dynamics');
        const lorenzCard = lorenzEntry ? lorenzEntry.closest('.card-link') : null;
        if (!lorenzCard) return;

        const plotDiv = lorenzCard.querySelector('#lorenz-plot');
        const playBtn = lorenzCard.querySelector('#lorenz-play-button');
        const stopBtn = lorenzCard.querySelector('#lorenz-stop-button');
        const resetBtn = lorenzCard.querySelector('#lorenz-reset-button');

        if (!plotDiv || !playBtn) return;

        if (plotDiv.hasAttribute('data-initialized')) return;
        plotDiv.setAttribute('data-initialized', 'true');

        // Lorenz parameters
        const sigma = 10, rho = 28, beta = 8/3;
        const dt = 0.02;

        const generateInitialConditions = () => {
            const conditions = [];
            for (let i = 0; i < 10; i++) {
                conditions.push({
                    x: (Math.random() - 0.5) * 2,
                    y: (Math.random() - 0.5) * 2, 
                    z: (Math.random() - 0.5) * 2 + 25
                });
            }
            return conditions;
        };

        let conditions = generateInitialConditions();
        let traces = [];

        const colors = [
            '#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd',
            '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf'
        ];

        const initializePlot = () => {
            traces = conditions.map((cond, i) => ({
                x: [cond.x],
                y: [cond.y], 
                z: [cond.z],
                type: 'scatter3d',
                mode: 'lines',
                line: { color: colors[i], width: 3 },
                name: `Trajectory ${i + 1}`
            }));

            const layout = {
                title: 'Lorenz Attractor - 10 Starting Conditions',
                scene: {
                    xaxis: { title: 'X', range: [-25, 25] },
                    yaxis: { title: 'Y', range: [-25, 25] },
                    zaxis: { title: 'Z', range: [0, 50] },
                    camera: { eye: { x: 1.5, y: 1.5, z: 1.5 } }
                },
                height: 600,
                margin: { l: 0, r: 0, b: 0, t: 50 }
            };

            Plotly.newPlot(plotDiv, traces, layout, { responsive: true });
        };

        const integrateLorenz = (state) => {
            const { x, y, z } = state;

            const dx1 = sigma * (y - x);
            const dy1 = x * (rho - z) - y;
            const dz1 = x * y - beta * z;

            const dx2 = sigma * ((y + 0.5 * dt * dy1) - (x + 0.5 * dt * dx1));
            const dy2 = (x + 0.5 * dt * dx1) * (rho - (z + 0.5 * dt * dz1)) - (y + 0.5 * dt * dy1);
            const dz2 = (x + 0.5 * dt * dx1) * (y + 0.5 * dt * dy1) - beta * (z + 0.5 * dt * dz1);

            const dx3 = sigma * ((y + 0.5 * dt * dy2) - (x + 0.5 * dt * dx2));
            const dy3 = (x + 0.5 * dt * dx2) * (rho - (z + 0.5 * dt * dz2)) - (y + 0.5 * dt * dy2);
            const dz3 = (x + 0.5 * dt * dx2) * (y + 0.5 * dt * dy2) - beta * (z + 0.5 * dt * dz2);

            const dx4 = sigma * ((y + dt * dy3) - (x + dt * dx3));
            const dy4 = (x + dt * dx3) * (rho - (z + dt * dz3)) - (y + dt * dy3);
            const dz4 = (x + dt * dx3) * (y + dt * dy3) - beta * (z + dt * dz3);

            return {
                x: x + (dt / 6) * (dx1 + 2*dx2 + 2*dx3 + dx4),
                y: y + (dt / 6) * (dy1 + 2*dy2 + 2*dy3 + dy4),
                z: z + (dt / 6) * (dz1 + 2*dz2 + 2*dz3 + dz4)
            };
        };

        const animateLorenz = () => {
            conditions = conditions.map(integrateLorenz);

            conditions.forEach((cond, i) => {
                traces[i].x.push(cond.x);
                traces[i].y.push(cond.y);
                traces[i].z.push(cond.z);

                if (traces[i].x.length > 1500) {
                    traces[i].x = traces[i].x.slice(-1000);
                    traces[i].y = traces[i].y.slice(-1000);
                    traces[i].z = traces[i].z.slice(-1000);
                }
            });

            Plotly.restyle(plotDiv, {
                x: traces.map(trace => trace.x),
                y: traces.map(trace => trace.y),
                z: traces.map(trace => trace.z)
            });
        };

        const playAnimation = () => {
            if (lorenzAnimationId) cancelAnimationFrame(lorenzAnimationId);

            playBtn.disabled = true;
            stopBtn.disabled = false;
            resetBtn.disabled = true;

            const animate = () => {
                animateLorenz();
                lorenzAnimationId = requestAnimationFrame(animate);
            };
            lorenzAnimationId = requestAnimationFrame(animate);
        };

        const stopAnimation = () => {
            if (lorenzAnimationId) {
                cancelAnimationFrame(lorenzAnimationId);
                lorenzAnimationId = null;
            }
            playBtn.disabled = false;
            stopBtn.disabled = true;
            resetBtn.disabled = false;
        };

        const resetAnimation = () => {
            stopAnimation();
            conditions = generateInitialConditions();
            initializePlot();
            playBtn.disabled = false;
            stopBtn.disabled = true;
            resetBtn.disabled = true;
        };

        playBtn.addEventListener('click', playAnimation);
        stopBtn.addEventListener('click', stopAnimation);
        resetBtn.addEventListener('click', resetAnimation);

        setTimeout(() => {
            initializePlot();
            stopBtn.disabled = true;
            resetBtn.disabled = true;
        }, 100);
    };

    // --- 5. Router and Main Page Logic ---
    const currentSlug = () => location.hash.replace(/^#/, '').trim();

    const showSlug = (slug) => {
        cards.forEach(c => {
            const id = c.querySelector('.entry')?.id;
            const match = id === slug;
            c.style.display = match ? 'block' : 'none';
        });

        if (slug === 'complex-morlet-wavelet-math') initializeMorletPlot();
        if (slug === 'lorenz-attractor-dynamics') initializeLorenzPlot();
    };

    const render = () => {
        const slug = currentSlug();
        const currentFilter = getCurrentFilter();
        
        // Update active filter button
        filterButtons.forEach(b => b.classList.remove('active'));
        const activeFilterBtn = document.querySelector(`.filter-btn[data-filter="${currentFilter}"]`);
        if (activeFilterBtn) activeFilterBtn.classList.add('active');

        if (slug) {
            showSlug(slug);
        } else {
            applyFilter(currentFilter); // Apply filter from URL
            initializeMorletPlot();
            initializeLorenzPlot();
        }
    };

    // --- 6. Card Link Navigation ---
    const cardLinks = document.querySelectorAll('.card-link');
    cardLinks.forEach(link => {
        const entry = link.querySelector('.entry');
        const entryId = entry?.id;
        
        link.removeAttribute('href');
        
        const clickableSymbol = document.createElement('span');
        clickableSymbol.className = 'entry-link-symbol';
        clickableSymbol.innerHTML = '🔗';
        clickableSymbol.style.cssText = `
            position: absolute;
            top: 1rem;
            right: 1rem;
            cursor: pointer;
            font-size: 1.2em;
            opacity: 0.7;
            transition: opacity 0.2s;
            pointer-events: auto;
            z-index: 10;
        `;
        
        clickableSymbol.addEventListener('mouseenter', () => {
            clickableSymbol.style.opacity = '1';
            clickableSymbol.style.transform = 'scale(1.1)';
        });
        clickableSymbol.addEventListener('mouseleave', () => {
            clickableSymbol.style.opacity = '0.7';
            clickableSymbol.style.transform = 'scale(1)';
        });
        
        clickableSymbol.addEventListener('click', (e) => {
            e.preventDefault();
            e.stopPropagation();
            const currentFilter = getCurrentFilter();
            updateURL(currentFilter, `#${entryId}`); // Preserve filter in URL
            showSlug(entryId);
        });
        
        link.style.position = 'relative';
        link.appendChild(clickableSymbol);
    });

    // --- 7. Event Listeners ---
    window.addEventListener('popstate', () => {
        if (lorenzAnimationId) {
            cancelAnimationFrame(lorenzAnimationId);
            lorenzAnimationId = null;
        }
        plotInitialized = false;
        render();
    });

    window.addEventListener('hashchange', render);

    // --- 8. Initial Render ---
    render();

    // --- 9. Lazy Load Videos ---
    const lazyLoadVideos = () => {
        const videoObserver = new IntersectionObserver((entries) => {
            entries.forEach(entry => {
                if (entry.isIntersecting) {
                    const video = entry.target;
                    // Remove preload and set autoplay
                    video.removeAttribute('preload');
                    video.setAttribute('autoplay', '');
                    video.setAttribute('playsinline', '');
                    video.muted = true;

                    // Try to play the video
                    const playPromise = video.play();
                    if (playPromise !== undefined) {
                        playPromise.catch(error => {
                            console.log('Autoplay prevented, adding controls');
                            video.setAttribute('controls', '');
                        });
                    }

                    videoObserver.unobserve(video);
                }
            });
        });

        document.querySelectorAll('.entry-video video').forEach(video => {
            videoObserver.observe(video);
        });
    };

    // Initialize lazy loading after DOM is ready
    document.addEventListener('DOMContentLoaded', lazyLoadVideos);
});