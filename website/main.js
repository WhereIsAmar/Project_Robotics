// Runs on page load
// Setup all event listeners here.
function onPageLoad() {
    //save
    let btn_save = document.querySelector('.js-save');
    btn_save.addEventListener('click', fun_save);
    //delete
    let btn_delete = document.querySelector('.js-delete');
    btn_delete.addEventListener('click', fun_delete);
    //save_hs
    let btn_save_hs = document.querySelector('.js-save-hs');
    btn_save_hs.addEventListener('click', fun_save_hs);
    //delete_hs
    let btn_delete_hs = document.querySelector('.js-delete-hs');
    btn_delete_hs.addEventListener('click', fun_delete_hs);
    //play
    let btn_play = document.querySelector('.js-play');
    btn_play.addEventListener('click', fun_play);
    //stop
    let btn_stop = document.querySelector('.js-stop');
    btn_stop.addEventListener('click', fun_stop);
}

function fun_save() {
    // how to make a server call
    fetch('http://localhost:8080/save')
        .then(() => console.log('fun_save'))
        .catch(err => alert('ther was an err'));
    alert('here1');
}
function fun_delete() {
    // how to make a server call
    fetch('http://localhost:8080/delete/1')
        .then(() => console.log('fun_delete'))
        .catch(err => alert('ther was an err'));
    alert('here2');
}
function fun_save_hs() {
    // how to make a server call
    fetch('http://localhost:8080/save')
        .then(() => console.log('fun_save_hs'))
        .catch(err => alert('ther was an err'));
    alert('here3');
}
function fun_delete_hs() {
    // how to make a server call
    fetch('http://localhost:8080/save')
        .then(() => console.log('fun_delete_hs'))
        .catch(err => alert('ther was an err'));
    alert('here4');
}
function fun_play() {
    // how to make a server call
    fetch('http://localhost:8080/save')
        .then(() => console.log('fun_play'))
        .catch(err => alert('ther was an err'));
    alert('here5');
}
function fun_stop() {
    // how to make a server call
    fetch('http://localhost:8080/save')
        .then(() => console.log('fun_stop'))
        .catch(err => alert('ther was an err'));
    alert('here6');
}



window.addEventListener('DOMContentLoaded', onPageLoad);
