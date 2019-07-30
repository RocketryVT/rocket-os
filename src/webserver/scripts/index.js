
let excluded_topics = [];
let tracked_topics = [];

function onRecieveData(data)
{
    let div = document.getElementById("data-readout");
    while (div.firstChild)
        div.removeChild(div.firstChild);

    for (let topic of excluded_topics)
        delete data[topic];

    tracked_topics = Object.keys(data);

    div.innerHTML = obj2table(data);
}

function isArrayOfPrimitives(object)
{
    if (!Array.isArray(object)) return false;
    for (let elem of object)
        if (elem === Object(elem)) return false;
    return true;
}

function obj2table(object)
{
    let html = '<table>';
    let keys = [];
    for (let key in object)
    {
        keys.push(key);
    }
    keys.sort();

    if (object !== Object(object))
    {
        let value = object.toString();
        if (typeof(object) == 'number') value = object.toFixed(6);
        html += '<tr><td>' + value + '</tr>'
    }

    for (let key of keys)
    {
        let item = object[key];
        let value = "";
        if (isArrayOfPrimitives(item))
            value = item.toString();
        else if (typeof(item) == 'number') value = item.toFixed(6);
        else value = (typeof(item) === 'object') ? obj2table(item) : item.toString();
        html += '<tr><td>' + key + '</td><td>' + value + '</tr>';
    }
    html += '</table>';
    return html;
}

const update_frequency = 4; // hz
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
