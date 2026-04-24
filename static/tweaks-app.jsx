// Apply palette/symbol tweaks from window.TWEAKS on load
(function () {
  const t = window.TWEAKS || {};
  if (t.palette)     document.body.setAttribute('data-palette',   t.palette);
  if (t.symbolStyle) document.body.setAttribute('data-symbols',   t.symbolStyle);
  if (t.waterAnim)   document.body.setAttribute('data-wateranim', t.waterAnim);
})();
