function select(event){
    item_num = this.item_num;
    document.querySelector("#item_" + item_num).selected = true;

    document.querySelectorAll(".item_icon").forEach(function(ele){ele.classList.remove("selected")});

    document.querySelector("#item_" + item_num + "_icon").classList.add("selected");
}


for(let i = 1; i <= 4; i++){
    ele = document.querySelector("#item_" + i + "_icon");
    ele.addEventListener("click", {item_num: i, handleEvent: select});
}