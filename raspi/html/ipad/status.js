/*
異常をipadに表示し、ブザーを鳴らす関数

引数：
    0：持ち去り検知　1：こじ開け検知

戻り値：
    なし
*/
function warning(num){
    if(num == 0){
        document.getElementById("status").innerText = "！持ち去りを検知しました！"
    }else if(num == 1){
        document.getElementById("status").innerText = "！こじ開けを検知しました！"
    }
    const audio = new Audio('sound.mp3');
    audio.play();
}