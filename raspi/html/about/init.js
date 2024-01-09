function onload(){
    hide_modal("modal_loading");
}

//document.addEventListener("DOMContentLoaded", onload);
document.querySelector("#main_pic").addEventListener('load', onload);
show_modal("modal_loading");