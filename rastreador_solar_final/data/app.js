// Sessão expirada em qualquer API → volta para o login.
const _fetch = window.fetch.bind(window);
window.fetch = async (...args) => {
  const r = await _fetch(...args);
  if (r.status === 401 && location.pathname !== '/login') location.href = '/login';
  return r;
};

// Navbar compartilhada — item ativo pelo pathname; Configurações só para admin.
(function () {
  const itens = [
    ['/',            '🏠', 'Dashboard'],
    ['/graficos',    '📈', 'Gráficos'],
    ['/performance', '⚡', 'Performance'],
    ['/logs',        '📋', 'Logs'],
    ['/config',      '⚙️', 'Configurações'],
    ['/changelog',   '📜', 'Changelog'],
    ['/sobre',       'ℹ️', 'Sobre'],
  ];
  const atual = location.pathname;
  const links = itens.map(([href, icone, label]) =>
    `<a href='${href}' class='nav-item${href === atual ? ' nav-ativo' : ''}'` +
    `${href === '/config' ? " id='navConfig' style='display:none'" : ''}>` +
    `<span class='nav-icon'>${icone}</span><span class='nav-label'>${label}</span></a>`
  ).join('');
  document.write(
    `<nav class='navbar'><div class='nav-brand'>☀️ <span>Solar ESP32</span></div>` +
    `<div class='nav-links'>${links}</div>` +
    `<div class='nav-direita'><span class='nav-perfil' id='navPerfil'></span>` +
    `<a href='/logout' class='nav-sair'>Sair</a></div></nav>`
  );
  fetch('/sessao').then(r => r.json()).then(s => {
    const badge = document.getElementById('navPerfil');
    if (s.perfil === 'adm') {
      badge.className = 'nav-perfil adm'; badge.textContent = '🔑 admin';
      const cfg = document.getElementById('navConfig');
      if (cfg) cfg.style.display = '';
    } else if (s.perfil === 'user') {
      badge.className = 'nav-perfil user'; badge.textContent = '👤 visitante';
    }
  }).catch(() => {});
})();

// Helpers de tempo compartilhados
function fmtRel(s) {
  return Math.floor(s / 3600) + 'h' + String(Math.floor(s % 3600 / 60)).padStart(2, '0');
}

// Recebe um registro do /historico e devolve rótulos com dia + hora.
// epoch > 0: data real (NTP). epoch = 0: tempo relativo (relógio não sincronizado).
function rotuloRegistro(r) {
  if (r.e > 0) {
    const d = new Date(r.e * 1000);
    return {
      dia:  d.toLocaleDateString('pt-BR', { day: '2-digit', month: '2-digit' }),
      hora: d.toLocaleTimeString('pt-BR', { hour: '2-digit', minute: '2-digit' }),
      full: d.toLocaleString('pt-BR', { day: '2-digit', month: '2-digit', year: 'numeric', hour: '2-digit', minute: '2-digit' }),
    };
  }
  return { dia: null, hora: fmtRel(r.t), full: '+' + fmtRel(r.t) + ' (relógio não sincronizado)' };
}
