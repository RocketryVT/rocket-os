
(() =>
{
    document.getElementById("console-input").childNodes[1]
        .addEventListener("keydown", handleInput);
})();


let command_history = [];
let command_index = command_history.length;

function handleInput(event)
{
    if (event.key == "Enter")
    {
        command(event.target.value);
        event.target.value = "";
        return;
    }
    if (command_history.length == 0) return;
    let input = document.getElementById("console-input").childNodes[1];
    if (event.key == "ArrowUp")
    {
        if (command_index > 0)
            command_index -= 1;
        let cmd = command_history[command_index];
        input.value = cmd;

    }
    else if (event.key == "ArrowDown")
    {
        if (command_index < command_history.length)
            command_index += 1;
        let cmd = command_history[command_index];
        if (cmd === undefined) cmd = "";
        input.value = cmd;
    }
}

let last_log_ts = 0;

function onRecieveData(data)
{
    let div = document.getElementById("data-readout");

    let new_logs = false;
    for (let log of data)
    {
        let ts = log.header.stamp.secs + log.header.stamp.nsecs / 1e9;
        if (last_log_ts >= ts) continue;

        last_log_ts = ts;
        let p = document.createElement("p");
        p.innerHTML = logstr(log);
        p.style.color = levelcolor(log.level);
        div.appendChild(p);
        new_logs = true;
    }

    if (new_logs)
    {
        let output = document.getElementById("data-readout");
        output.scrollTop = output.scrollHeight;
    }
}

function levelstr(level)
{
    switch (level)
    {
        case 1: return "DEBUG";
        case 2: return " INFO";
        case 4: return " WARN";
        case 8: return "ERROR";
        case 16: return "FATAL";
    }
    return " ??? ";
}

function levelcolor(level)
{
    switch (level)
    {
        case 1: return "green";
        case 2: return "black";
        case 4: return "orange";
        case 8: return "red";
        case 16: return "red";
    }
    return "black";
}

function logstr(log)
{
    return "<b>[" + levelstr(log.level) + "] [" +
        log.header.stamp.secs.toString().padStart(10, '0') + "." +
        log.header.stamp.nsecs.toString().padStart(9, '0') + "] [" +
        log.name + "]:</b> " + log.msg;
}

function command(string)
{
    if (string != command_history[command_history.length - 1])
        command_history.push(string);
    command_index = command_history.length;

    let req = new XMLHttpRequest();
    req.onload = function(e)
    {
        console.log("Successfully sent.");
    };
    req.open('POST', '/command', true);
    req.send(string);
}

function getRosout()
{
    let req = new XMLHttpRequest();
    req.onload = function(e)
    {
        let array =
        onRecieveData(JSON.parse(e.target.response));
    };
    req.open('POST', '/rosout', true);
    req.send();
}

const update_frequency = 1; // hz
const update_period = 1/update_frequency; // seconds

const update_loop = window.setInterval(function()
{
    let req = new XMLHttpRequest();
    req.onload = function(e)
    {
        let data = JSON.parse(e.target.response);
        for (let key in data)
        {
            data[key] = JSON.parse(data[key]);
        }
        onRecieveData(data);
    };
    req.open('POST', '/rosout', true);
    req.send();
},
update_period*1000);
