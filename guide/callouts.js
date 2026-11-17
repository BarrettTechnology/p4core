// Numbered callouts: one badge + arrow per item, placed against its anchor's real box
// (a widget hotspot, a menu-bar label or a menu entry) after layout, before printing.
document.fonts.ready.then(function () {
  var problems = [];
  // 1. fit: shrink the panel text, then the screenshots, until each page's body fits
  document.querySelectorAll('.page').forEach(function (pg) {
    var main = pg.querySelector('main');
    var fs = 1, shot = 1;
    for (var k = 0; k < 40 && main.scrollHeight > main.clientHeight + 1; k++) {
      if (fs > 0.8) fs -= 0.02; else if (shot > 0.7) shot -= 0.02; else if (fs > 0.68) fs -= 0.02; else break;
      pg.style.setProperty('--fs', fs.toFixed(2)); pg.style.setProperty('--shot', shot.toFixed(2));
    }
    if (main.scrollHeight > main.clientHeight + 1)
      problems.push('page ' + (Array.prototype.indexOf.call(document.querySelectorAll('.page'), pg) + 1) +
                    ': the content does not fit, even shrunk (shorten it or split the page)');
  });
  // 2. callouts
  var GAP = 14, B = 22;
  document.querySelectorAll('.fig').forEach(function (fig) {
    var data = fig.querySelector('script.callouts');
    var items = data ? JSON.parse(data.textContent) : [];
    if (!items.length) return;
    var fr = fig.getBoundingClientRect();
    var svg = document.createElementNS('http://www.w3.org/2000/svg', 'svg');
    svg.setAttribute('class', 'arrows');
    svg.setAttribute('width', fr.width); svg.setAttribute('height', fr.height);
    fig.appendChild(svg);
    var placed = [];
    items.forEach(function (it) {
      var a = null;
      fig.querySelectorAll('[data-anchor]').forEach(function (el) {
        if (!a && el.getAttribute('data-anchor') === it.anchor) a = el;
      });
      if (!a) { problems.push('callout ' + it.n + ': no anchor ' + it.anchor); return; }
      var r = a.getBoundingClientRect();
      var box = (fig.querySelector('.win') || fig.querySelector('.menu') || fig.querySelector('.imgfig')).getBoundingClientRect();
      var bl = box.left - fr.left, br = box.right - fr.left;
      var x0 = r.left - fr.left, y0 = r.top - fr.top, w = r.width, h = r.height;
      var side = it.side || (it.anchor.indexOf('menu:') === 0 ? 'above'
                 : it.anchor.indexOf('item:') === 0 ? 'right' : 'left');
      var bx, by, tx, ty;
      var pt = it.anchor.indexOf('pt:') === 0 || it.anchor.indexOf('img:') === 0;   // photos: next to the point
      if (side === 'left' || side === 'row') { bx = (pt ? x0 : Math.min(x0, bl)) - GAP - B; by = y0 + h / 2 - B / 2;
                              ty = y0 + h / 2; tx = side === 'row' ? bl + 2 : x0; }   // row: point at the labelled row
      if (side === 'right') { bx = (pt ? x0 + w : Math.max(x0 + w, br)) + GAP; by = y0 + h / 2 - B / 2; tx = x0 + w; ty = y0 + h / 2; }
      if (side === 'above') { bx = x0 + w / 2 - B / 2; by = y0 - GAP - B; tx = x0 + w / 2; ty = y0; }
      if (side === 'below') { bx = x0 + w / 2 - B / 2; by = y0 + h + GAP; tx = x0 + w / 2; ty = y0 + h; }
      // keep badges from overlapping: push along the side's free axis
      for (var guard = 0; guard < 8; guard++) {
        var hit = placed.some(function (p) { return Math.abs(p[0] - bx) < B + 2 && Math.abs(p[1] - by) < B + 2; });
        if (!hit) break;
        if (side === 'left' || side === 'row') bx -= B + 4; else if (side === 'right') bx += B + 4; else bx += B + 3;
      }
      placed.push([bx, by]);
      var b = document.createElement('div');
      b.className = 'badge'; b.textContent = it.n;
      b.style.left = bx + 'px'; b.style.top = by + 'px';
      fig.appendChild(b);
      var cx = bx + B / 2, cy = by + B / 2;
      var sx = side === 'left' ? bx + B : side === 'right' ? bx : cx;
      var sy = side === 'above' ? by + B : side === 'below' ? by : cy;
      var line = document.createElementNS('http://www.w3.org/2000/svg', 'line');
      line.setAttribute('x1', sx); line.setAttribute('y1', sy);
      line.setAttribute('x2', tx); line.setAttribute('y2', ty);
      line.setAttribute('marker-end', 'url(#ah)');
      svg.appendChild(line);
    });
    svg.insertAdjacentHTML('afterbegin', '<defs><marker id="ah" viewBox="0 0 10 10" refX="9" refY="5" ' +
      'markerWidth="5" markerHeight="5" orient="auto"><path d="M0 0L10 5L0 10z" fill="#14286E"/></marker></defs>');
  });
  // badges and arrows must stay on the page
  document.querySelectorAll('.page').forEach(function (pg, i) {
    var pr = pg.getBoundingClientRect();
    pg.querySelectorAll('.badge').forEach(function (b) {
      var r = b.getBoundingClientRect();
      if (r.left < pr.left || r.right > pr.right || r.top < pr.top || r.bottom > pr.bottom)
        problems.push('page ' + (i + 1) + ': callout ' + b.textContent + ' is off the page (change its :side)');
    });
  });
  document.body.setAttribute('data-problems', problems.join('|'));
});
