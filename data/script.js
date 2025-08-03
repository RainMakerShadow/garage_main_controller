function updateData() {
    const xhr = new XMLHttpRequest();
    xhr.open("POST", "/get_data", true);
    //xhr.withCredentials = true;
    xhr.onreadystatechange = function () {
        if (xhr.readyState == 4 && xhr.status == 200) {
            try {
                const response = JSON.parse(xhr.responseText);
                //document.getElementById("ssid").value = response.ssid;
                document.getElementById("wifi_mode").value = response.wifi_mode;
                document.getElementById("mqtt_server_address").value =
                    response.mqtt_server;
                document.getElementById("mqtt_server_port").value =
                    response.mqtt_port;
                document.getElementById("mqtt_user").value = response.mqtt_user;
                document.getElementById("client_id").value = response.clientId;
                document.getElementById("login").value = response.login;
                var select = document.getElementById("network_list");
                response.wifi_networks.forEach(function (network) {
                    var option = document.createElement("option");
                    option.value = network;
                    option.textContent = network;
                    select.appendChild(option);
                });
                var select = document.getElementById("wifi_mode");
                response.wifi_mode.forEach(function (mode) {
                    var option = document.createElement("option");
                    option.value = mode;
                    option.textContent = mode;
                    select.appendChild(option);
                });
            } catch (error) {
                console.error("Parsing error JSON:", error);
                document.getElementById("response").innerText =
                    "Error: incorrect JSON";
            }
        }
    };
    xhr.send();
}

document.getElementById("save_wifi_set").addEventListener("click", function () {
    const xhr = new XMLHttpRequest();
    xhr.open("POST", "/save_wifi_set", true);
    xhr.setRequestHeader("Content-Type", "application/x-www-form-urlencoded");
    xhr.onload = function () {
        if (xhr.status >= 200 && xhr.status < 300) {
            console.log("Answer:", xhr.responseText);
        } else {
            console.error("Error:", xhr.status, xhr.statusText);
        }
    };

    xhr.onerror = function () {
        console.error("Network error");
    };
    var data =
        "ssid=" +
        document.getElementById("network_list").value +
        "&wifi_pass=" +
        document.getElementById("wifi_pass").value +
        "&wifi_mode=" +
        document.getElementById("wifi_mode").value;
    xhr.send(data);
});

document.getElementById("save_mqtt_set").addEventListener("click", function () {
    const xhr = new XMLHttpRequest();
    xhr.open("POST", "/save_mqtt_set", true);
    xhr.setRequestHeader("Content-Type", "application/x-www-form-urlencoded");
    xhr.onload = function () {
        if (xhr.status >= 200 && xhr.status < 300) {
            console.log("Answer:", xhr.responseText);
        } else {
            console.error("Error:", xhr.status, xhr.statusText);
        }
    };

    xhr.onerror = function () {
        console.error("Network error");
    };
    var data =
        "mqtt_server=" +
        document.getElementById("mqtt_server_address").value +
        "&mqtt_port=" +
        document.getElementById("mqtt_server_port").value +
        "&mqtt_user=" +
        document.getElementById("mqtt_user").value +
        "&mqtt_pass=" +
        document.getElementById("mqtt_pass").value +
        "&clientId=" +
        document.getElementById("client_id").value;
    xhr.send(data);
});

document.getElementById("save_user_set").addEventListener("click", function () {
    const xhr = new XMLHttpRequest();
    xhr.open("POST", "/save_user_set", true);
    xhr.setRequestHeader("Content-Type", "application/x-www-form-urlencoded");
    xhr.onload = function () {
        if (xhr.status >= 200 && xhr.status < 300) {
            console.log("Answer:", xhr.responseText);
        } else {
            console.error("Error:", xhr.status, xhr.statusText);
        }
    };

    xhr.onerror = function () {
        console.error("Network error");
    };
    var data =
        "login=" +
        document.getElementById("login").value +
        "&pass=" +
        document.getElementById("pass").value;
    xhr.send(data);
});
