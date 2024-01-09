function onload(){
    hide_modal("modal_loading");
}

show_modal("modal_loading");

//document.addEventListener("DOMContentLoaded", onload);
if(document.querySelector("#main_pic").complete){
    onload();
}else{
    document.querySelector("#main_pic").addEventListener('load', onload);
}
