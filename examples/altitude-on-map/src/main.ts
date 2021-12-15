import "./style.css";

// const app = document.querySelector<HTMLDivElement>('#app')!

// app.innerHTML = `
//   <h1>Hello Vite!</h1>
//   <a href="https://vitejs.dev/guide/features.html" target="_blank">Documentation</a>
// `

let circle = {
  radius: 1,
  loaction: {
    x: 1,
    y: 1,
  },
  draw: () => {
    console.log("draw");
  },
};

circle.draw();

console.log("hello");
