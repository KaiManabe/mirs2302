function onload(){
    let request = new XMLHttpRequest();

    request.onreadystatechange = function(){
    if (request.readyState == 4){
        if (request.status == 200){
        let data = request.responseText;
            if(data == "open"){
                window.location.href = "../main/";
            }
        }
    }
    }

    request.open('GET', 'isopen.php', true);
    request.send();
}

onload();