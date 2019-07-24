
function onRecieveData(data)
{
    let div = document.getElementById("data-readout");
    while (div.firstChild)
        div.removeChild(div.firstChild);
    for (let topic in data)
    {
        let p = document.createElement("p");
        div.appendChild(p);
        p.innerHTML = "<b>" + topic + ":</b><br>" + data[topic];
    }
}

const update_frequency = 20; // hz
const update_period = 1/20; // seconds

const update_loop = window.setInterval(function()
{
    let req = new XMLHttpRequest();
    req.onload = function(e)
    {
        let data = JSON.parse(e.target.response);
        onRecieveData(data);
    };
    req.open('POST', '/update', true);
    req.send();
},
update_period*1000);
