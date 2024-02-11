let slideIndex = [1, 1, 1]; // An array to hold the slide index for each carousel
const carouselClasses = ['softwareTeam', 'hardwareTeam', 'outreachTeam']; // Class names for each carousel

// Initialize all carousels
carouselClasses.forEach((className, i) => showSlides(slideIndex[i], i));

function plusSlides(n, carouselNo) {
  slideIndex[carouselNo] += n;
  showSlides(slideIndex[carouselNo], carouselNo);
}

function showSlides(n, carouselNo) {
  let i;
  let slides = document.getElementsByClassName(carouselClasses[carouselNo]);
  if (n > slides.length) { slideIndex[carouselNo] = 1; }
  if (n < 1) { slideIndex[carouselNo] = slides.length; }
  for (i = 0; i < slides.length; i++) {
      slides[i].style.display = "none";
  }
  slides[slideIndex[carouselNo]-1].style.display = "block";
}

// You may need to adjust the interval setup for each carousel if needed
