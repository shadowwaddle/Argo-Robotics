function plusSlides(n, slideshowId) {
    showSlides(slideIndex[slideshowId] += n, slideshowId);
}
  
function showSlides(n, slideshowId) {
    let i;
    const slides = document.querySelectorAll(`.${slideshowId} .my-slide`);
    if (n > slides.length) {
      slideIndex[slideshowId] = 1;
    }
    if (n < 1) {
      slideIndex[slideshowId] = slides.length;
    }
    for (i = 0; i < slides.length; i++) {
      slides[i].style.display = "none";
    }
    slides[slideIndex[slideshowId] - 1].style.display = "block";
}
  
// Automatic slideshow
setInterval(() => {
    plusSlides(1, 'slideshow-1');
    plusSlides(1, 'slideshow-2');
    plusSlides(1, 'slideshow-3');
    // Add more slideshow instances as needed
}, 5000); // Change slide every 5 seconds (adjust as needed)
  