import { App } from './src/app.js';

// Browsers (Firefox notably) cache number-input values across reloads —
// edits to HTML `value="…"` defaults silently lose to the previous session.
// Force every number input back to its HTML defaultValue before App boots
// so editing the HTML actually changes what users see on reload. Saved
// tunings (localStorage-backed) survive because they're applied later via
// loadTuning(), not via input restoration.
function resetNumberInputsToHtmlDefaults() {
	for (const el of document.querySelectorAll('input[type=number]')) {
		el.value = el.defaultValue;
	}
}

window.addEventListener('DOMContentLoaded', () => {
	resetNumberInputsToHtmlDefaults();
	window.app = new App();
});
