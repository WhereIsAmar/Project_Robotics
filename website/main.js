// Runs on page load
// Setup all event listeners here.
function onPageLoad() {
    //save
    let btns = document.querySelectorAll('.js-playback');
    btns.forEach(btn => btn.addEventListener('click', playback));
}

function playback(event) {
    const name = event.target.textContent;
    fetch(`http://localhost:8080/playback/${name}`)
        .then(() => alert('done'));
}


window.addEventListener('DOMContentLoaded', onPageLoad);
