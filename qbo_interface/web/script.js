const ws = new WebSocket("ws://" + location.host + "/ws");

ws.onmessage = event => {
    const data = JSON.parse(event.data);
    const states = data.states || [];
    const current = data.current || "";
    const container = document.getElementById("graph");
    container.innerHTML = "";

    for (const state of states) {
        const div = document.createElement("div");
        div.className = "state-bubble" + (state === current ? " active" : "");
        div.innerText = state;
        container.appendChild(div);
    }
};