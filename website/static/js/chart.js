// I know that, I said that I would do it with python, but after looking it up, I realized that making the entire chart in JS was easier.
// because it was already "in" the HTML and I wouldn't have to suffer with the integration.
// I am kinda sad, but I don't have enough time to make it nicely in Python :(
const ctx = document.getElementById('speedChart').getContext('2d');
const speedChart = new Chart(ctx, {
    type: 'line',
    data: {
        labels: [],
        datasets: [{
            label: 'Speed (km/h)',
            data: [], // for the speed data
            borderColor: 'blue',
            tension: 0.1
        }]
    },
    options: {
        responsive: true,
        scales: {
            x: {
                type: 'realtime',
                realtime: {
                    duration: 20000, // last 20s
                    refresh: 1000, // refresh every 1s
                    delay: 2000, // but delayed but 2s
                    onRefresh: chart => { // start graph on refresh / reset
                        const now = Date.now();
                        chart.data.datasets.forEach(dataset => {
                            dataset.data.push({
                                x: now,
                                y: data.speed
                            });
                        });
                    }
                }
            },
            y: {
                title: {
                    display: true,
                    text: 'Speed (km/h)'
                }
            }
        }
    }
});

const eventSource = new EventSource('/stream');
let data = { speed: 0 };

eventSource.onmessage = function(event) {
    const receivedData = JSON.parse(event.data);
    data.speed = receivedData.speed; // get speed specifically
    // Manually update the chart when new data is received
    speedChart.update();
};