
let excluded_topics = [];
let tracked_topics = [];

let last_log_secs = 0;
let last_log_nsecs = 0;

function onRecieveData(data)
{
    let div = document.getElementById("data-readout");
    let log = data["/rosout"];
    if (last_log_secs == log.header.stamp.secs &&
        last_log_nsecs == log.header.stamp.nsecs) return;

    last_log_secs = log.header.stamp.secs;
    last_log_nsecs = log.header.stamp.nsecs;

    let p = document.createElement("p");
    p.textContent = logstr(log);
    div.appendChild(p);
}

function logstr(log)
{
    return "[" + log.header.stamp.secs +
        "." + log.header.stamp.nsecs + "] [" +
        log.name + "]: " + log.msg;
}

function command(string)
{
    let req = new XMLHttpRequest();
    req.onload = function(e)
    {
        console.log("Successfully sent.");
    };
    req.open('POST', '/command', true);
    req.send(string);
}

const update_frequency = 5; // hz
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
    req.open('POST', '/update', true);
    req.send();
},
update_period*1000);
